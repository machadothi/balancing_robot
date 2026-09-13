#!/usr/bin/env python3
"""Check the relative links in the Markdown docs.

For every [text](path#anchor) link to a local file:
  - the file must exist;
  - a #Lnn anchor must be inside the file, and when the link text names a
    symbol (`name()` or `NAME`), that symbol must appear on the anchored line;
  - a #heading anchor into a Markdown file must match one of its headings.

With --fix, #Lnn anchors whose symbol moved are re-pointed at the symbol's
definition (the first line that starts with it, or failing that the first line
containing it).

Usage: scripts/check_docs.py [--fix]      (exit code 1 if problems remain)
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = [ROOT / "README.md", ROOT / "TODO.md", ROOT / "test" / "README.md", *sorted((ROOT / "docs").glob("*.md"))]

LINK = re.compile(r"\[([^\]]*)\]\(([^)\s]+)\)")
SYMBOL = re.compile(r"`?([A-Za-z_][A-Za-z0-9_]*)(?:\(\))?`?")


def github_slug(heading):
    slug = heading.strip().lower()
    slug = re.sub(r"[^\w\- ]", "", slug)
    return slug.replace(" ", "-")


def headings(path):
    return {github_slug(m.group(1)) for m in re.finditer(r"^#+\s+(.*)$", path.read_text(), re.M)}


def is_comment(line):
    return line.lstrip().startswith(("//", "*", "/*"))


def find_definition(lines, name):
    word = re.compile(r"\b" + re.escape(name) + r"\b")
    n = re.escape(name)
    # A definition: a #define, the end of a typedef, or the name after a type
    for i, line in enumerate(lines):
        if is_comment(line):
            continue
        if re.match(r"^\s*#define\s+" + n + r"\b", line) or re.match(r"^\}\s*" + n + r"\s*;", line):
            return i + 1
        if re.match(r"^[\w\s\*]*\b" + n + r"\s*(\(|=|\s|$)", line):
            if not line.rstrip().endswith(";") or "=" in line:
                return i + 1
    # Otherwise the first use outside a comment
    for i, line in enumerate(lines):
        if word.search(line) and not is_comment(line):
            return i + 1
    return None


def check(fix):
    problems = 0
    for doc in DOCS:
        if not doc.exists():
            continue
        text = doc.read_text()
        changed = False

        def visit(m):
            nonlocal problems, changed
            label, target = m.group(1), m.group(2)
            if re.match(r"^[a-z]+:", target) or target.startswith("#") and doc.suffix != ".md":
                return m.group(0)
            path_part, _, anchor = target.partition("#")
            dest = (doc.parent / path_part).resolve() if path_part else doc
            where = f"{doc.relative_to(ROOT)}: [{label}]({target})"

            if not dest.exists():
                print(f"{where}: missing file")
                problems += 1
                return m.group(0)
            if not anchor:
                return m.group(0)

            if re.fullmatch(r"L\d+(-L\d+)?", anchor):
                lines = dest.read_text().splitlines()
                line_no = int(anchor[1:].split("-")[0])
                sym = SYMBOL.fullmatch(label.split(",")[0].strip())
                name = sym.group(1) if sym and ("`" in label or "()" in label) else None
                in_range = 1 <= line_no <= len(lines)
                ok = in_range and (name is None or name in lines[line_no - 1])
                if ok:
                    return m.group(0)
                new_line = find_definition(lines, name) if name else None
                if fix and new_line:
                    changed = True
                    return f"[{label}]({path_part}#L{new_line})"
                print(f"{where}: " + ("line out of range" if not in_range else f"`{name}` not on line {line_no}"))
                problems += 1
                return m.group(0)

            if dest.suffix == ".md" and anchor not in headings(dest):
                print(f"{where}: no such heading")
                problems += 1
            return m.group(0)

        new_text = LINK.sub(visit, text)
        if changed:
            doc.write_text(new_text)
    return problems


if __name__ == "__main__":
    fix = "--fix" in sys.argv[1:]
    n = check(fix)
    if fix and n:
        # Re-run to report only what the fix could not resolve
        n = check(False)
    print(f"{n} problem(s)")
    sys.exit(1 if n else 0)

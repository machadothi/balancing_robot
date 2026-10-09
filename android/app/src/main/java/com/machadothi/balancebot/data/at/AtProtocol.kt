package com.machadothi.balancebot.data.at

/** A complete reply of the robot's AT console (src/cmd/at_cmd.c) */
sealed interface AtReply {
    /** AT+X=v or AT+X accepted */
    data object Ok : AtReply

    /** AT+X? answered with "+X:value" */
    data class Value(val name: String, val value: String) : AtReply

    /** "ERROR:n" (n = firmware result code) or "ERROR:text" */
    data class Error(val code: Int?, val text: String) : AtReply {
        val meaning: String get() = AtProtocol.errorMeaning(code, text)
    }

    /** No complete reply in time */
    data object Timeout : AtReply
}

object AtProtocol {
    private const val PROMPT = "> "

    /** Firmware result codes, AT_Result_t in src/cmd/at_cmd.h */
    fun errorMeaning(code: Int?, text: String): String = when (code) {
        1 -> "command failed"
        2 -> "unknown command"
        3 -> "invalid value"
        4 -> "value out of range"
        5 -> "not ready"
        6 -> "busy"
        else -> text
    }

    /**
     * Strip CR and leading "> " prompts.
     * @return null for lines that carry no reply: blank lines and telemetry records
     */
    fun clean(raw: String): String? {
        var line = raw.replace("\r", "")
        while (line.startsWith(PROMPT)) line = line.removePrefix(PROMPT)
        line = line.trim()
        return if (line.isEmpty() || line.startsWith("seq:")) null else line
    }

    /**
     * The reply a cleaned line completes, or null for anything else (the echo of
     * the command on USB, the banner, log lines)
     */
    fun parseFinal(line: String): AtReply? = when {
        line == "OK" -> AtReply.Ok
        line.startsWith("ERROR:") -> line.removePrefix("ERROR:").let { AtReply.Error(it.toIntOrNull(), it) }
        line.startsWith("+") && line.contains(':') -> {
            val colon = line.indexOf(':')
            AtReply.Value(line.substring(1, colon), line.substring(colon + 1))
        }
        else -> null
    }
}

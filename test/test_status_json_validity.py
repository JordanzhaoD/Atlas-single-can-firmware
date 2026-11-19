"""Gates for the /status JSON appenders and the deployed dashboard blob.

Born from the 2026-11-19 dual-CAN beta18 field lesson, ported here after the
same bug shipped in single-CAN v2.0.0: appendClone27DiagJson (as ported from
beta17) emitted a stray close-quote before ,"tx": which broke the ENTIRE
/status payload — the native suites never exercise the JSON emitters, and a
text-grep contract cannot see quote pairing. This module:

1. Emulates the `j +=` concatenation of each status-JSON appender with dummy
   values and json.loads()es the result — a literal-punctuation bug now fails
   CI instead of a field session.
2. Decompresses the DEPLOYED dashboard blob (DASH_HTML_GZ) and pins the
   clone27 switch: present, and positioned above the NAG DND area (the
   v2.10 relocation from Jordan's field feedback).
"""

import json
import re
import unittest
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DASH = ROOT / "include" / "web" / "mcp2515_dashboard.h"
UI_GEN = ROOT / "include" / "web" / "mcp2515_dashboard_ui.h"


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    end = source.index("\n}", brace)
    return source[brace + 1 : end]


def _emulate(body: str) -> str:
    """Replay the j += / appendXxx lines with dummy values, return the fragment."""
    out = []
    for raw in body.splitlines():
        line = raw.strip()
        if not line or line.startswith("//") or line.startswith("const "):
            continue
        m = re.fullmatch(r'j \+= R"JSON\((.*)\)JSON";', line)
        if m:
            out.append(m.group(1))
            continue
        m = re.fullmatch(r'j \+= "((?:[^"\\]|\\.)*)";', line)
        if m:
            out.append(m.group(1).encode().decode("unicode_escape"))
            continue
        m = re.fullmatch(r"j \+= (\S+) \? \"true\" : \"false\";", line)
        if m:
            out.append("true")
            continue
        m = re.fullmatch(r"j \+= \S+ \? String\([^)]*\) : String\([^)]*\);", line)
        if m:
            out.append("0")
            continue
        m = re.fullmatch(r"appendTxEvidenceJson\(j, [^,]+, [^)]+\);", line)
        if m:
            out.append("{}")
            continue
        m = re.fullmatch(r"j \+= [A-Za-z_][A-Za-z0-9_:]*\([^;]*\);", line)
        if m:  # name-helper call (dashLegacy045Clone27StateName & co.)
            out.append("x")
            continue
        m = re.fullmatch(r"j \+= [A-Za-z_][A-Za-z0-9_.>\[\]]+;", line)
        if m:  # scalar field reference (numbers, String fields)
            out.append("0")
            continue
        raise AssertionError(f"emulator cannot parse appender line: {line!r}")
    return "".join(out)


def _decompress_blob() -> str:
    src = UI_GEN.read_text(encoding="utf-8")
    m = re.search(r"DASH_HTML_GZ\[\].*?=\s*\{(.*?)\};", src, re.S)
    assert m, "DASH_HTML_GZ array not found in generated header"
    data = bytes(int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]{2})", m.group(1)))
    return zlib.decompress(data, 16 + zlib.MAX_WBITS).decode("utf-8", "replace")


class StatusJsonValidityTests(unittest.TestCase):
    """Every /status appender must assemble parseable JSON."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.dash = DASH.read_text(encoding="utf-8")

    def _assert_valid(self, signature: str, key: str) -> None:
        body = _function_body(self.dash, signature)
        fragment = _emulate(body)
        # Status appenders are mid-object by design: they open with a
        # leading comma that continues the enclosing /status object.
        payload = json.loads("{" + fragment.lstrip(",") + "}")
        self.assertIn(key, payload)

    def test_clone27_diag_fragment_is_valid_json(self) -> None:
        """v2.0.0 shipped the beta17 stray-quote bug here; it must never return."""
        self._assert_valid(
            "static void appendClone27DiagJson(String &j, unsigned long now)",
            "clone27Diag",
        )

    def test_wheel_dnd_diag_fragment_is_valid_json(self) -> None:
        """Control: the older wheel-DND appender stays valid too."""
        self._assert_valid(
            "static void appendWheelDndDiagJson(String &j, unsigned long now)",
            "wheelDndDiag",
        )


class DeployedBlobContractTests(unittest.TestCase):
    """The served page (decompressed DASH_HTML_GZ) must carry the clone27
    switch, above the NAG DND area (v2.10 relocation)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.blob = _decompress_blob()

    def test_clone27_toggle_exists_in_deployed_blob(self) -> None:
        # The minifier strips attribute quotes (id=def-clone27-tgl), so pin
        # the bare tokens, not the source markup form.
        self.assertIn("def-clone27-tgl", self.blob)
        self.assertIn("clone27_dnd:c27Tgl&&c27Tgl.checked?'1':'0',", self.blob)
        self.assertIn("c27Tgl.checked=!!d.clone27_dnd", self.blob)

    def test_clone27_toggle_sits_above_the_nag_dnd_area(self) -> None:
        toggle = self.blob.index("def-clone27-tgl")
        nag_area = self.blob.index("nag-mode-select")
        self.assertLess(
            toggle,
            nag_area,
            "clone27 switch must render above the NAG DND controls (v2.10 field fix)",
        )


if __name__ == "__main__":
    unittest.main()

"""Run production controllers with ENCODER fallback rows without changing user routes."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    source = (root / "Core/Src/control/search_lateral_control.c").read_text()
    for side, array in [("RIGHT", "search_right_fallback_commands"), ("LEFT", "search_left_fallback_commands")]:
        start = source.index("static const char *const " + array)
        begin = source.index("{", start)
        end = source.index("};", begin)
        source = source[:begin] + '{\n    "LATERAL,' + side + ',ENCODER",\n' + source[end:]
    fixture = directory / "search_fixture.c"
    fixture.write_text(source)
    binary = directory / "test"
    subprocess.run(["clang", "-std=c11", "-fsanitize=address,undefined", "-ffunction-sections", "-fdata-sections", "-Itests/host", "-ICore/Inc", "tests/host/test_ir_travel_encoder.c", "Core/Src/control/straight_control.c", "Core/Src/control/motion_heading.c", "Core/Src/control/motion_log.c", "Core/Src/control/lateral_control.c", str(fixture), "-Wl,-dead_strip", "-lm", "-o", str(binary)], cwd=root, check=True)
    subprocess.run([str(binary)], cwd=root, check=True)

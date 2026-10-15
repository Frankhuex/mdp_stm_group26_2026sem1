"""Exercise signed ENCODER offsets through production fallback parsing."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    source = (root / "Core/Src/control/search_lateral_control.c").read_text()
    for side, offset, array in [("RIGHT", 7, "search_right_fallback_commands"), ("LEFT", -7, "search_left_fallback_commands")]:
        begin = source.index("{", source.index("static const char *const " + array))
        end = source.index("};", begin)
        source = source[:begin] + '{\n    "LATERAL,' + side + ',ENCODER,' + str(offset) + '",\n' + source[end:]
    fixture = directory / "search.c"
    fixture.write_text(source)
    prefix = (root / "tests/host/test_search_lateral_fallback.c").read_text().split("static void trigger(void)")[0]
    test = directory / "test.c"
    test.write_text(prefix + """
int main(void) {
    for(uint8_t side=0;side<2;side++) {
        setup(1000);Straight_IRTravelMm=1072;
        assert(SearchLateral_Start(side,50,50)==0);
        encoder_delta=16000;poll();encoder_delta=0;
        assert(status().fallback && SearchLateral_State()==SEARCH_RUNNING);
        lateral_status_t l;Lateral_Status(&l);
        assert(l.from_encoder && l.first_angle_offset_deg==(side?7:-7));
        assert(angles[0]==l.angle_deg+(side?7:-7));
        turn_state=TURN_DONE;poll();
        assert(Lateral_State()==LATERAL_RUNNING);
        Lateral_Status(&l);assert(l.stage==3 && l.middle_mm==286);
        encoder_delta=5000;poll();encoder_delta=0;
        for(int i=0;i<20 && turn_calls<2;i++)poll();
        assert(turn_calls==2 && angles[1]==l.angle_deg);
        turn_state=TURN_DONE;poll();assert(SearchLateral_State()==SEARCH_DONE);
    }
    puts("PASS: fallback parses signed ENCODER offsets and completes both arcs");
}
""")
    binary = directory / "test"
    subprocess.run(["clang", "-std=c11", "-fsanitize=address,undefined", "-ffunction-sections", "-fdata-sections", "-Itests/host", "-ICore/Inc", str(test), "Core/Src/control/straight_control.c", "Core/Src/control/motion_heading.c", "Core/Src/control/motion_log.c", "Core/Src/control/lateral_control.c", str(fixture), "-Wl,-dead_strip", "-lm", "-o", str(binary)], cwd=root, check=True)
    subprocess.run([str(binary)], cwd=root, check=True)

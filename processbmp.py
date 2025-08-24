import pathlib
import struct


imagedata = pathlib.Path("borisinky.bmp").read_bytes()
signature, file_size, reserved1, reserved2, data_offset = struct.unpack(
    "<2sIHHI", imagedata[:14]
)


def rle():
    current_colour = 0
    run_length = 0
    runs = []
    for byte in imagedata[data_offset:]:
        if byte != current_colour:
            runs.append(run_length)
            run_length = 0

        current_colour = byte
        run_length += 1

    if run_length > 0:
        runs.append(run_length)

    print(runs)


final_bytes = []
for a, b, c, d, e, f, g, h in zip(*([iter(imagedata[data_offset:])] * 8)):
    final_bytes.append(
        (
            (a & 0x1) << 7
            | (b & 0x1) << 6
            | (c & 0x1) << 5
            | (d & 0x1) << 4
            | (e & 0x1) << 3
            | (f & 0x1) << 2
            | (g & 0x1) << 1
            | (h & 0x1)
        ).to_bytes(1)
    )

pathlib.Path("src/boris.bin").write_bytes(b"".join(final_bytes))

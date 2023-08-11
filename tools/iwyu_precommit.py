# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: Apache-2.0

import sys
import subprocess
from pathlib import Path
import re

REPO_ROOT_DIR = Path(__file__).resolve().parent.parent


def execute_command(command: str, check_error_code: bool = False):
    sp_child = subprocess.run(command,
                              shell=True,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT)
    if check_error_code and sp_child.returncode:
        sys.stderr.buffer.write(sp_child.stdout)
        sys.exit(sp_child.returncode)
    return sp_child.stdout.decode('utf-8')


def generate_iwyu_mappings():
    script = REPO_ROOT_DIR / "tools" / "generate_iwyu_mappings.py"
    execute_command(f"python3 {script}", check_error_code=True)


def gather_fixes_include_what_you_use(output_file: Path, files: str):
    compile_commands_json = REPO_ROOT_DIR / "build/compile_commands.json"
    mapping_file = REPO_ROOT_DIR / "build/iwyu_mappings/main.imp"
    iwyu_tool_arguments = f"--jobs 8 --verbose -p {compile_commands_json} {files}"
    iwyu_arguments = f"-Xiwyu --no_fwd_decls -Xiwyu --mapping_file={mapping_file} -Xiwyu --max_line_length=180 -Xiwyu --update_comments -Xiwyu --prefix_header_includes=keep"

    with open(output_file, 'w') as f:
        output = execute_command(
            f"iwyu_tool {iwyu_tool_arguments} -- {iwyu_arguments} &> {output_file}"
        )
        # This is specific for this code base, where we use angle brackets according to https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
        # Unfortunatly include-what-you-use has no config option for this yet: https://github.com/include-what-you-use/include-what-you-use/issues/496
        output = re.sub(r'#include\s+"([^"]+)"', r'#include <\1>', output)
        output.replace("#include <",
                       "#include <").replace("#include \"", "#include \"")
        f.write(output)

    if "failed:" in output or "error:" in output:
        sys.stderr.buffer.write(str.encode(output))
        sys.stderr.buffer.write(b"Precommit failed, see output above")
        sys.exit(1)


def apply_fixes_from_include_what_you_use(output_file: Path):
    execute_command(
        f"fix_include --nosafe_headers --noreorder < {output_file}")


def format_files():
    # We format everything and not just the given files, because iwyu also makes changes in included header files, which were not given in the file list
    execute_command(f"pre-commit run clang-format --all-files")


def main(argv: list[str] = sys.argv):
    files = [Path(file) for file in argv[1:] if file.endswith(".cpp")]
    if not files:
        return
    files_string = " ".join(str(file) for file in files)

    output_file = REPO_ROOT_DIR / "build/iwyu_output.txt"

    generate_iwyu_mappings()
    gather_fixes_include_what_you_use(output_file, files_string)
    apply_fixes_from_include_what_you_use(output_file)
    format_files()


if __name__ == "__main__":
    main()

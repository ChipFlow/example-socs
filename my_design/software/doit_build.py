import os
import shutil
from pathlib import Path
from doit.action import CmdAction
from doit import create_after
import chipflow.config

CHIPFLOW_SOFTWARE_DIR = chipflow.config.get_dir_software()
BUILD_DIR = "./build/software"
DESIGN_DIR = os.path.dirname(__file__) + "/.."
# We set the SSH_DIR so that we don't mount the user's .ssh folder into dockcross
DOCKCROSS_CMD = f"SSH_DIR=\"/dev/null\" {CHIPFLOW_SOFTWARE_DIR}/dockcross-linux-riscv32"
RISCVGCC = f"{DOCKCROSS_CMD} riscv32-unknown-linux-gnu-gcc"
RISCVOBJCOPY = f"{DOCKCROSS_CMD} riscv32-unknown-linux-gnu-objcopy"
CINC = f"-I. -I{BUILD_DIR}"
LINKER_SCR = f"{BUILD_DIR}/generated/sections.lds"
SOFTWARE_START = f"{BUILD_DIR}/generated/start.S"
CFLAGS = f"-g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,{LINKER_SCR},--strip-debug -static -ffreestanding -nostdlib {CINC}"

def task_gather_depencencies():
    src_files = []
    target_files = []

    # Project dependencies
    rel_paths = _get_source_rel_paths(f"{DESIGN_DIR}/software", ["*.*"])
    for rel_path in rel_paths:
        src_files.append(f"{DESIGN_DIR}/software{rel_path}")
        target_files.append(f"{BUILD_DIR}/{rel_path}")

    # ChipFlow lib dependencies
    rel_paths = _get_source_rel_paths(f"{CHIPFLOW_SOFTWARE_DIR}/drivers", ["*.h", "*.c", "*.S"])
    for rel_path in rel_paths:
        src_files.append(f"{CHIPFLOW_SOFTWARE_DIR}/drivers{rel_path}")
        target_files.append(f"{BUILD_DIR}/drivers{rel_path}")

    def copy_files():
        _create_build_dir()
        for i in range(len(src_files)):
            shutil.copyfile(src_files[i - 1], target_files[i - 1])

    return {
        "actions": [(copy_files)],
        "file_dep": src_files,
        "targets": target_files,
        "verbosity": 2
    }  

@create_after(executed="gather_depencencies", target_regex='.*/software\.elf')
def task_build_software_elf():
    sources = [SOFTWARE_START]
    sources += _gather_source_paths(f"{BUILD_DIR}/drivers", ["*.c", "*.S"])
    sources += _gather_source_paths(f"{BUILD_DIR}", ["*.c"])

    sources_str = " ".join(sources)
    
    return {
        "actions": [f"{RISCVGCC} {CFLAGS} -o {BUILD_DIR}/software.elf {sources_str}"],
        "file_dep": sources + [LINKER_SCR],
        "targets": [f"{BUILD_DIR}/software.elf"],
        "verbosity": 2
    }

@create_after(executed="build_software_elf", target_regex='.*/software\.bin')
def task_build_software():
    return {
        "actions": [f"{RISCVOBJCOPY} -O binary {BUILD_DIR}/software.elf {BUILD_DIR}/software.bin"],
        "file_dep": [f"{BUILD_DIR}/software.elf"],
        "targets": [f"{BUILD_DIR}/software.bin"],
    }

def _create_build_dir():
    Path(f"{BUILD_DIR}/drivers").mkdir(parents=True, exist_ok=True)

def _get_source_rel_paths(source_dir, globs):
    abs_source_dir = str(Path(source_dir).absolute())
    rel_paths = []
    for glob in globs:
        source_paths = list(Path(abs_source_dir).glob(glob))
        for source_path in source_paths:
            dst = str(source_path).replace(abs_source_dir, "")
            rel_paths.append(dst)  

    return rel_paths

def _gather_source_paths(source_dir, globs):
    sources = []
    for glob in globs:
        source_paths = list(Path(source_dir).glob(glob))
        for source_path in source_paths:
            sources.append(f"{source_dir}/" + str(source_path.name))   

    return sources

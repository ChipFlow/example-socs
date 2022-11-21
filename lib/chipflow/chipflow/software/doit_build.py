import os
import shutil
from pathlib import Path
from doit.action import CmdAction

CHIPFLOW_SOFTWARE_DIR = os.path.dirname(__file__)
BUILD_DIR = "./build/software"
# We set the SSH_DIR so that we don't mount the user's .ssh folder into dockcross
DOCKCROSS_CMD = f"SSH_DIR=\"/dev/null\" {CHIPFLOW_SOFTWARE_DIR}/dockcross-linux-riscv32"
RISCVGCC = f"{DOCKCROSS_CMD} riscv32-unknown-linux-gnu-gcc"
RISCVOBJCOPY = f"{DOCKCROSS_CMD} riscv32-unknown-linux-gnu-objcopy"
CINC = f"-I. -I{BUILD_DIR}"
LINKER_SCR = f"{BUILD_DIR}/generated/sections.lds"
SOFTWARE_START = f"{BUILD_DIR}/generated/start.S"
CFLAGS = f"-g -march=rv32ima -mabi=ilp32 -Wl,--build-id=none,-Bstatic,-T,{LINKER_SCR},--strip-debug -static -ffreestanding -nostdlib {CINC}"

def task_set_params():
    def return_params(design_dir):
        return {
            "design_dir": design_dir
        }

    return {
        "actions": [return_params],
        "params": [
            {
                "name": "design_dir",
                "default": "",
                "short": "d"
            },
        ],
    }

def task_gather_chipflow_library_deps():   
    def copy_files():
        _create_build_dir()
        source_dir = f"{CHIPFLOW_SOFTWARE_DIR}/drivers/"
        target_dir = f"{BUILD_DIR}/drivers/"
        _copy_files(source_dir, target_dir, ["*.h", "*.c", "*.S"])

    return {
        "actions": [(copy_files)],
        "uptodate": [False], # TODO: Improve this so it looks at source/dest files
        "verbosity": 2
    }    

def task_gather_project_deps():
    def copy_files(design_dir):
        _create_build_dir()
        source_dir = f"{design_dir}/software/"
        target_dir = f"{BUILD_DIR}/"
        _copy_files(source_dir, target_dir, ["*.*"])

    return {
        "actions": [(copy_files)],
        "getargs": {"design_dir": ("set_params", "design_dir")},
        "uptodate": [False], # TODO: Improve this so it looks at source/dest files
        "verbosity": 2
    }  

def task_build_software_elf():
    def get_build_cmd():
        sources = _gather_source_paths(f"{BUILD_DIR}/drivers", ["*.c", "*.S"])
        sources += _gather_source_paths(f"{BUILD_DIR}", ["*.c"])
        
        sources_src = " ".join(sources)

        return f"{RISCVGCC} {CFLAGS} -o {BUILD_DIR}/software.elf {SOFTWARE_START} {sources_src}"
    
    return {
        "actions": [CmdAction(get_build_cmd)],
        "file_dep": [
            SOFTWARE_START,
            # TODO: Dynamically look at sources to see if they are changed
            LINKER_SCR
        ],
        "targets": [f"{BUILD_DIR}/software.elf"],
        "verbosity": 2
    }


def task_build_software_bin():
    return {
        "actions": [f"{RISCVOBJCOPY} -O binary {BUILD_DIR}/software.elf {BUILD_DIR}/software.bin"],
        "file_dep": [f"{BUILD_DIR}/software.elf"],
        "targets": [f"{BUILD_DIR}/software.bin"],
    }

def _create_build_dir():
    Path(f"{BUILD_DIR}/drivers").mkdir(parents=True, exist_ok=True)

def _copy_files(source_dir, target_dir, globs):
        files = []
        for glob in globs:
            files += list(Path(source_dir).glob(glob))      
        
        for file in files:
            src = file
            dst = str(file).replace(source_dir, target_dir)
            shutil.copyfile(src, dst)

def _gather_source_paths(source_dir, globs):
    sources = []
    for glob in globs:
        source_paths = list(Path(source_dir).glob(glob))
        for source_path in source_paths:
            sources.append(f"{source_dir}/" + str(source_path.name))   

    return sources

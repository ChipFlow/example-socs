# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
from pathlib import Path
import shutil

import yowasp_yosys
import chipflow_lib.config
from doit.action import CmdAction


CHIPFLOW_MODEL_DIR = chipflow_lib.config.get_dir_models()
DESIGN_DIR = os.path.dirname(__file__) + "/.."
YOSYS_DATDIR = os.path.abspath(
    os.path.dirname(yowasp_yosys.__file__)) + "/share"
BUILD_DIR = "./build/sim"
CXX = f"{sys.executable} -m ziglang c++"
CXXFLAGS = f"-O3 -g -std=c++17 -I {CHIPFLOW_MODEL_DIR}"
RTL_CXXFLGAGS = "-O1 -std=c++17"
# TODO: we need these models to be pulled in according to what has been used in the design
DESIGN_MODELS = ["uart", "spiflash", "wb_mon", "log"]


def task_build_sim_soc_c_files():
    return {
        "actions": [f"cd {BUILD_DIR} && poetry run yowasp-yosys sim_soc.ys"],
        "targets": [f"{BUILD_DIR}/sim_soc.cc", f"{BUILD_DIR}/sim_soc.h"],
        "file_dep": [f"{BUILD_DIR}/sim_soc.ys", f"{BUILD_DIR}/sim_soc.il"],
    }


def task_build_sim_soc_objects():
    def get_build_cmd():
        cmd = f"{CXX} -I . -I {YOSYS_DATDIR}/include {RTL_CXXFLGAGS} "
        cmd += f"-o {BUILD_DIR}/sim_soc.o -c {BUILD_DIR}/sim_soc.cc"

        return cmd

    return {
        "actions": [CmdAction(get_build_cmd)],
        "targets": [f"{BUILD_DIR}/sim_soc.o"],
        "file_dep": [f"{BUILD_DIR}/sim_soc.cc", f"{BUILD_DIR}/sim_soc.h"],
    }


def task_gather_sim_model_files():
    model_src_files = []
    model_target_files = []
    for model in DESIGN_MODELS:
        model_src_files.append(f"{CHIPFLOW_MODEL_DIR}/{model}.cc")
        model_target_files.append(f"{BUILD_DIR}/models/{model}.cc")

    model_header_src_files = []
    model_header_target_files = []
    for model in DESIGN_MODELS:
        model_header_src_file = f"{CHIPFLOW_MODEL_DIR}/{model}.h"
        if os.path.exists(model_header_src_file):
            model_header_src_files.append(model_header_src_file)
            model_header_target_files.append(f"{BUILD_DIR}/models/{model}.h")

    def do_copy():
        # Ensure model dir exists
        Path(f"{BUILD_DIR}/models").mkdir(parents=True, exist_ok=True)

        for i in range(len(model_src_files)):
            shutil.copyfile(model_src_files[i - 1], model_target_files[i - 1])

        for i in range(len(model_header_src_files)):
            shutil.copyfile(
                model_header_src_files[i - 1],
                model_header_target_files[i - 1]
            )

    return {
        "actions": [do_copy],
        "targets": model_target_files + model_header_target_files,
        "file_dep": model_src_files + model_header_src_files,
    }


def task_gather_sim_project_files():
    src_files = []
    target_files = []
    sources = ["main"]

    for source in sources:
        src_files.append(f"{DESIGN_DIR}/sim/{source}.cc")
        target_files.append(f"{BUILD_DIR}/{source}.cc")

    def do_copy():
        # Ensure dir exists
        Path(f"{BUILD_DIR}").mkdir(parents=True, exist_ok=True)

        for i in range(len(src_files)):
            shutil.copyfile(src_files[i - 1], target_files[i - 1])

    return {
        "actions": [do_copy],
        "file_dep": src_files,
        "targets": target_files,
    }


def task_build_sim_model_objects():
    for model in DESIGN_MODELS:
        model_obj_file = f"{BUILD_DIR}/models/{model}.o"

        cmd = f"{CXX} -I . -I {YOSYS_DATDIR}/include {CXXFLAGS} -o {model_obj_file} "
        cmd += f"-c {BUILD_DIR}/models/{model}.cc"

        yield {
            "name": model_obj_file,
            "actions": [cmd],
            "targets": [model_obj_file],
            "file_dep": [f"{BUILD_DIR}/models/{model}.cc", f"{BUILD_DIR}/sim_soc.h"],
        }


def task_build_sim_objects():
    sources = ["main"]

    for source in sources:
        obj_file = f"{BUILD_DIR}/{source}.o"
        yield {
            "name": obj_file,
            "actions": [f"{CXX} -I . -I {YOSYS_DATDIR}/include {CXXFLAGS} -o {obj_file} -c {BUILD_DIR}/{source}.cc"],
            "targets": [obj_file],
            "file_dep": [f"{BUILD_DIR}/{source}.cc", f"{BUILD_DIR}/sim_soc.h"],
        }


def task_build_sim():
    exe = ".exe" if os.name == "nt" else ""

    model_object_paths = []
    for model in DESIGN_MODELS:
        model_object_paths.append(f"{BUILD_DIR}/models/{model}.o")

    model_object_paths_str = " ".join(model_object_paths)

    def get_build_cmd():
        return f"{CXX} -o {BUILD_DIR}/sim_soc{exe} {BUILD_DIR}/sim_soc.o {model_object_paths_str} {BUILD_DIR}/main.o"

    return {
        "actions": [CmdAction(get_build_cmd)],
        "targets": [f"{BUILD_DIR}/sim_soc{exe}"],
        "file_dep": [f"{BUILD_DIR}/sim_soc.o", f"{BUILD_DIR}/main.o"] + model_object_paths,
    }

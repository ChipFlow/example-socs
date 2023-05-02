# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.steps.software import SoftwareStep
from ..software import doit_build


class MySoftwareStep(SoftwareStep):
    doit_build_module = doit_build

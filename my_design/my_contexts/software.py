# SPDX-License-Identifier: BSD-2-Clause

from chipflow_lib.contexts.software import SoftwareContext
from ..software import doit_build


class MySoftwareContext(SoftwareContext):
    doit_build_module = doit_build

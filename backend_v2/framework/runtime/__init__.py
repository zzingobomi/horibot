"""framework/runtime — Module lifecycle + DI + Transport wiring."""

from framework.runtime.api import ModuleRuntime
from framework.runtime.app import Runtime
from framework.runtime.lifecycle import Lifecycle

__all__ = ["ModuleRuntime", "Runtime", "Lifecycle"]

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain
from conan.tools.files import collect_libs

class CDTConan(ConanFile):
    name = "cdt"
    version = "1.3.0"
    license = "MPL-2.0 License"
    url = "https://github.com/artem-ogre/CDT"
    description = "Numerically robust C++ implementation of constrained Delaunay triangulation (CDT)"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps"
    options = {
        "shared": [True, False],
        "use_boost": [True, False],
        "as_compiled_library": [True, False],
        "enable_testing": [True, False],
    }
    default_options = {
        "shared": False,
        "use_boost": False,
        "as_compiled_library": False,
    }
    exports_sources = "*", "!.idea", "!conanfile.py"

    def requirements(self):
        if self.options.use_boost:
            self.requires("boost/1.83.0")
        self.requires("catch2/3.4.0")

    def configure(self):
        if self.options.use_boost:
            self.options["boost"].header_only = True

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["CDT_USE_BOOST"] = self.options.use_boost
        tc.cache_variables["CDT_USE_AS_COMPILED_LIBRARY"] = self.options.as_compiled_library
        tc.cache_variables["CMAKE_PROJECT_CDT_INCLUDE"] = "conan_basic_setup.cmake"
        tc.cache_variables["CDT_ENABLE_TESTING"] = self.options.enable_testing
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if self.options.enable_testing:
            cmake.test(cli_args=["--verbose"])

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = collect_libs(self)

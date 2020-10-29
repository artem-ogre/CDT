# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from conans import ConanFile, CMake, tools


class CDTConan(ConanFile):
    name = "CDT"
    version = "1.0.0"
    license = "MPL-2.0 License"
    url = "https://github.com/artem-ogre/CDT"
    description = "Numerically robust C++ implementation of constrained Delaunay triangulation (CDT)"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "use_boost": [True, False],
        "as_compiled_library": [True, False],
    }
    default_options = {
        "shared": False,
        "use_boost": False,
        "as_compiled_library": False,
    }
    generators = "cmake"
    exports_sources = "*", "!.idea", "!conanfile.py"

    def requirements(self):
        if self.options.use_boost:
            self.requires.add("boost/1.71.0")

    def configure(self):
        if self.options.use_boost:
            self.options["boost"].header_only = True

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["CDT_USE_BOOST"] = self.options.use_boost
        cmake.definitions["CDT_USE_AS_COMPILED_LIBRARY"] = self.options.as_compiled_library
        cmake.definitions["CMAKE_PROJECT_CDT_INCLUDE"] = "conan_basic_setup.cmake"
        cmake.configure()
        return cmake

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout, CMakeDeps
from conan.tools.build import valid_min_cppstd

required_conan_version = ">=2.0"

class CDTConan(ConanFile):
    name = "cdt"
    package_type = "library"
    version = "1.4.5"
    license = "MPL-2.0 License"
    url = "https://github.com/artem-ogre/CDT"
    description = "Numerically robust C++ implementation of constrained Delaunay triangulation (CDT)"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "header_only": [True, False],
        "enable_callback_handler": [True, False],
        "enable_testing": [True, False],
        "no_exceptions": [True, False],
        "use_64_bit_index_type": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "header_only": True,
        "enable_callback_handler": False,
        "enable_testing": False,
        "no_exceptions": False,
        "use_64_bit_index_type": False,
    }
    exports_sources = "*", "!.idea", "!conanfile.py"

    @property
    def _needs_boost(self):
        #Versions below C++11 require boost
        return not valid_min_cppstd(self, "11")
    
    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC
    
    def configure(self):
        if self._needs_boost:
            self.options["boost"].header_only = True

        if self.options.get_safe("shared") or self.options.header_only:
            self.options.rm_safe("fPIC")
        if self.options.header_only:
            self.options.rm_safe("shared")

    def requirements(self):
        if self._needs_boost:
            self.requires("boost/[^1.78.0]")        

    def build_requirements(self):
        self.requires("catch2/3.4.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["CDT_USE_BOOST"] = self._needs_boost
        tc.variables["CDT_USE_AS_COMPILED_LIBRARY"] = (
            not self.options.header_only
        )
        tc.variables["CDT_ENABLE_CALLBACK_HANDLER"] = self.options.enable_callback_handler
        tc.variables["CDT_ENABLE_TESTING"] = self.options.enable_testing
        tc.variables["CDT_DISABLE_EXCEPTIONS"] = self.options.no_exceptions
        tc.variables["CDT_USE_64_BIT_INDEX_TYPE"] = self.options.use_64_bit_index_type
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

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
        self.cpp_info.set_property("cmake_file_name", "CDT")
        self.cpp_info.set_property("cmake_target_name", f"CDT::CDT")
        self.cpp_info.set_property("pkg_config_name",  "CDT")

        if self._needs_boost:
            self.cpp_info.requires = ["boost::boost"]
            self.cpp_info.defines.append("CDT_USE_BOOST")
        if not self.options.header_only:
            self.cpp_info.defines.append("CDT_USE_AS_COMPILED_LIBRARY")
        if self.options.enable_callback_handler:
            self.cpp_info.defines.append("CDT_ENABLE_CALLBACK_HANDLER")
        if self.options.no_exceptions:
            self.cpp_info.defines.append("CDT_DISABLE_EXCEPTIONS")
        if self.options.use_64_bit_index_type:
            self.cpp_info.defines.append("CDT_USE_64_BIT_INDEX_TYPE")

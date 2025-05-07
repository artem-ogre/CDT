# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain


class CDTConan(ConanFile):
    name = "cdt"
    version = "1.4.1"
    license = "MPL-2.0 License"
    url = "https://github.com/artem-ogre/CDT"
    description = "Numerically robust C++ implementation of constrained Delaunay triangulation (CDT)"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "without_boost": [True, False],
        "without_catch2": [True, False],
        "without_doxygen": [True, False],
        "without_qt": [True, False],
        "minify_qt": [True, False],
    }
    default_options = {
        "without_boost": True,
        "without_catch2": False,
        "without_doxygen": True,
        "without_qt": True,
        "minify_qt": True,
    }
    exports_sources = "*", "!.idea", "!conanfile.py"

    def requirements(self):
        if not self.options.without_boost:
            self.requires("boost/1.83.0")
        if not self.options.without_qt:
            self.requires("qt/6.7.3")

    def build_requirements(self):
        if not self.options.without_catch2:
            self.requires("catch2/3.8.0")
        if not self.options.without_doxygen:
            self.requires("doxygen/1.13.2")

    def configure(self):
        if not self.options.without_qt:
            self.options["qt"].shared = True
            if self.options.minify_qt:
                self.options["qt"].openssl = False
                self.options["qt"].with_icu = False
                self.options["qt"].with_harfbuzz = False
                self.options["qt"].with_libjpeg = False
                self.options["qt"].with_libpng = False
                self.options["qt"].with_sqlite3 = False
                self.options["qt"].with_pq = False
                self.options["qt"].with_odbc = False
                self.options["qt"].with_brotli = False
                self.options["qt"].with_openal = False
                self.options["qt"].with_md4c = False

    def generate(self):
        tc = CMakeToolchain(self)
        tc.user_presets_path = False  # disable generating CMakeUserPresets.json
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        print(
            "Building with conan not supported by this recipe. Please call cmake directly."
        )

    def package(self):
        print(
            "Installing with conan not supported by this recipe. Please call cmake directly."
        )

    def package_info(self):
        print("Packaging with conan not supported by this recipe.")

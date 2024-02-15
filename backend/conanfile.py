from conans import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conans.tools import Git
import os


class CarlaViz(ConanFile):
    name = "carlaviz"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "simulator": ["carla"],
        "frontend": ["xviz"],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "simulator": "carla",
        "frontend": "xviz"
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = (
        "src/*",
        "include/*",
        "CMakeLists.txt",
    )

    def _configure_cmake(self) -> CMake:
        cmake = CMake(self)
        variables = {
            "CARLAVIZ_VERSION": f"{self.version}"
        }
        variables["CARLAVIZ_SIMULATOR"] = str(self.options.simulator).upper()
        variables["CARLAVIZ_FRONTEND"] = str(self.options.frontend).upper()
        cmake.configure(variables=variables)
        return cmake

    def set_version(self):
        try:
            if "CARLAVIZ_CI_VERSION_OVERRIDE" in os.environ:
                self.version = os.environ["CARLAVIZ_CI_VERSION_OVERRIDE"]
            else:
                self.version = (
                    Git(folder=self.recipe_folder).run("describe --tags --abbr=0") or "0.0.1"
                )
        except:
            self.version = "0.0.1"

    def requirements(self):
        self.requires("gflags/2.2.2")  # for cmd option parse
        self.requires("spdlog/1.12.0")
        self.requires("fmt/9.1.0")
        if self.options.simulator == "carla":
            self.requires("carla/0.9.15")
        if self.options.frontend == "xviz":
            self.requires("xviz/0.5.1")
            # require GeoJson
            self.requires("nlohmann_json/3.11.2")
            self.requires("websocketpp/0.8.2")
            # require lodepng
            self.requires("lodepng/cci.20200615")

    def configure(self):
        if self.options.frontend == "xviz":
            self.options["websocketpp"].asio = "standalone"
            self.options["websocketpp"].with_openssl = False

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        dp = CMakeDeps(self)
        dp.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = self._configure_cmake()
        if self.should_build:
            cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

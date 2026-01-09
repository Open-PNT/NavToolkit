
project('aspn-lcm', 'c')

aspn_lcm_inc = include_directories('c/include', is_system : true)

required_lcm_version = '>=1.3.1'
lcm_dep = dependency('lcm', version: required_lcm_version, required: false)

# LCM doesn't reliably ship with a pkg-config file. Check whether an
# appropriate version is in the default header/library path.
if not lcm_dep.found()
    c_compiler = meson.get_compiler('c')
    if c_compiler.has_header('lcm/lcm.h')
        prefix = '#include <lcm/lcm.h>'
        lcm_major = c_compiler.get_define('LCM_MAJOR_VERSION', prefix: prefix)
        lcm_minor = c_compiler.get_define('LCM_MINOR_VERSION', prefix: prefix)
        lcm_micro = c_compiler.get_define('LCM_MICRO_VERSION', prefix: prefix)
        lcm_version = lcm_major + '.' + lcm_minor + '.' + lcm_micro
        if lcm_version.contains('..')
            error('lcm.h is missing LCM_*_VERSION defs')
        elif not lcm_version.version_compare(required_lcm_version)
            error('lcm version ' + lcm_version + ' is not ' +
                    required_lcm_version)
        else
            lcm_dep = c_compiler.find_library('lcm')
        endif
    endif
endif

if not lcm_dep.found()
    # Could not find lcm installed, attempt to build subproject with cmake
    cmake = import('cmake')
    subproj_options = cmake.subproject_options()
    subproj_options.append_compile_args('c', '-w') # Inhibit all warnings
    # CMAKE_EXPORT_NO_PACKAGE_REGISTRY prevents CMake from adding the
    # subproject build folder to the local package registry. This should
    # prevent CMake from "discovering" the LCM build folder and should cause
    # Meson to keep treating it like a subproject instead of a system library.
    # LCM_ENABLE_JAVA will allow building of lcm-spy and lcm-logplayer-gui;
    # however they may need to be built separately.  See instructions below.
    subproj_options.add_cmake_defines({
        'CMAKE_EXPORT_NO_PACKAGE_REGISTRY': true,
        'LCM_ENABLE_EXAMPLES': false,
        'LCM_ENABLE_GO': false,
        'LCM_ENABLE_JAVA': false,
        'LCM_ENABLE_LUA': false,
        'LCM_ENABLE_PYTHON': false,
        'LCM_ENABLE_TESTS': false,
    })

    # To build lcm-logplayer-gui: Modify subprojects/lcm-generated/meson.build
    # to set LCM_ENABLE_JAVA to true. Then from the NavToolkit root folder:
    # ninja -C build
    # ninja -C build/subprojects/lcm/__CMake_build
    # ln -s build/subprojects/lcm/__CMake_build/lcm-java/lcm-logplayer-gui build/
    # chmod u+x build/lcm-logplayer-gui
    # ./build/lcm-logplayer-gui &
    # Note: lcm-logplayer-gui may need to be edited to remove the `-Xincgc` argument

    lcm_subproj = cmake.subproject('lcm', options: subproj_options)
    lcm_dep = lcm_subproj.dependency('lcm')
endif

aspn_cpp_lcm_inc = include_directories('cpp', is_system : true)
aspn_lcm_srcs = ['c/src/datasources_lcm_messages_aspn_types_geodeticposition3d_type.c',
                    'c/src/datasources_lcm_messages_aspn_gnss.c',
                    'c/src/datasources_lcm_messages_aspn_gpsephemeris.c',
                    'c/src/datasources_lcm_messages_aspn_tdoatoknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_nested_tdoatoknownfeature_tdoa_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_meta_gpsephemeris.c',
                    'c/src/datasources_lcm_messages_aspn_speed.c',
                    'c/src/datasources_lcm_messages_aspn_meta_rawopticalcameraimage.c',
                    'c/src/datasources_lcm_messages_aspn_altitude.c',
                    'c/src/datasources_lcm_messages_aspn_geodeticposition2d.c',
                    'c/src/datasources_lcm_messages_aspn_overhausermagnetometer.c',
                    'c/src/datasources_lcm_messages_aspn_attitude2d.c',
                    'c/src/datasources_lcm_messages_aspn_platformtype.c',
                    'c/src/datasources_lcm_messages_aspn_directiontoknownfeature3d.c',
                    'c/src/datasources_lcm_messages_aspn_nested_rangeratetoknownfeature_range_rate_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_nested_deltarangetoknownfeature_delta_range_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_bearingtoknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_correspondedopticalcamerafeatures.c',
                    'c/src/datasources_lcm_messages_aspn_tdoatounknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_types_timestamp.c',
                    'c/src/datasources_lcm_messages_aspn_rangeratetoknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_sf.c',
                    'c/src/datasources_lcm_messages_aspn_directiontounknownfeature2d.c',
                    'c/src/datasources_lcm_messages_aspn_types_mounting.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_gm1.c',
                    'c/src/datasources_lcm_messages_aspn_deltarotation2d.c',
                    'c/src/datasources_lcm_messages_aspn_deltaposition2d.c',
                    'c/src/datasources_lcm_messages_aspn_directiontoknownfeature2d.c',
                    'c/src/datasources_lcm_messages_aspn_discreteevent.c',
                    'c/src/datasources_lcm_messages_aspn_rangetounknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_meta_ins.c',
                    'c/src/datasources_lcm_messages_aspn_deltaposition3d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_generic3d.c',
                    'c/src/datasources_lcm_messages_aspn_deltarangetoknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_clock2.c',
                    'c/src/datasources_lcm_messages_aspn_meta_imu.c',
                    'c/src/datasources_lcm_messages_aspn_nested_bearingtoknownfeature_bearing_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_attitude3d.c',
                    'c/src/datasources_lcm_messages_aspn_temperature.c',
                    'c/src/datasources_lcm_messages_aspn_deltavelocity1d.c',
                    'c/src/datasources_lcm_messages_aspn_eventlog.c',
                    'c/src/datasources_lcm_messages_aspn_types_metadataheader.c',
                    'c/src/datasources_lcm_messages_aspn_accumulateddistancetraveled.c',
                    'c/src/datasources_lcm_messages_aspn_positionattitude.c',
                    'c/src/datasources_lcm_messages_aspn_deltarotation1d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_threeaxismagnetometer.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_clock2.c',
                    'c/src/datasources_lcm_messages_aspn_meta_overhausermagnetometer.c',
                    'c/src/datasources_lcm_messages_aspn_directiontounknownfeature3d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_gnss.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_uniform.c',
                    'c/src/datasources_lcm_messages_aspn_deltarotation3d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_sf.c',
                    'c/src/datasources_lcm_messages_aspn_uncorrespondedopticalcamerafeatures.c',
                    'c/src/datasources_lcm_messages_aspn_positionvelocity.c',
                    'c/src/datasources_lcm_messages_aspn_nested_gnss_gnss_obs.c',
                    'c/src/datasources_lcm_messages_aspn_imu.c',
                    'c/src/datasources_lcm_messages_aspn_velocity1d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_error_model.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_error_model.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_error_model.c',
                    'c/src/datasources_lcm_messages_aspn_attitude1d.c',
                    'c/src/datasources_lcm_messages_aspn_streamingvideo.c',
                    'c/src/datasources_lcm_messages_aspn_nested_directiontoknownfeature2d_direction_2d_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_sf.c',
                    'c/src/datasources_lcm_messages_aspn_rangeratetounknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_meta_generic1d.c',
                    'c/src/datasources_lcm_messages_aspn_ins.c',
                    'c/src/datasources_lcm_messages_aspn_geodeticposition3d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_bias.c',
                    'c/src/datasources_lcm_messages_aspn_types_header.c',
                    'c/src/datasources_lcm_messages_aspn_deltaposition1d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_uniform.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_clock2.c',
                    'c/src/datasources_lcm_messages_aspn_rangetoknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_nested_rangetoknownfeature_range_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_gm1.c',
                    'c/src/datasources_lcm_messages_aspn_meta_opticalcamerafeatures.c',
                    'c/src/datasources_lcm_messages_aspn_directionofmotion3d.c',
                    'c/src/datasources_lcm_messages_aspn_nested_directiontoknownfeature3d_direction_3d_to_feat_obs.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic2d_bias.c',
                    'c/src/datasources_lcm_messages_aspn_deltavelocity2d.c',
                    'c/src/datasources_lcm_messages_aspn_bearingtounknownfeature.c',
                    'c/src/datasources_lcm_messages_aspn_threeaxismagnetometer.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_bias.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic1d_gm1.c',
                    'c/src/datasources_lcm_messages_aspn_meta_generic2d.c',
                    'c/src/datasources_lcm_messages_aspn_meta_nested_generic3d_uniform.c',
                    'c/src/datasources_lcm_messages_aspn_deltavelocity3d.c',
                    'c/src/datasources_lcm_messages_aspn_velocity2d.c',
                    'c/src/datasources_lcm_messages_aspn_barometricpressure.c',
                    'c/src/datasources_lcm_messages_aspn_deltarange.c',
                    'c/src/datasources_lcm_messages_aspn_positionvelocityattitude.c',
                    'c/src/datasources_lcm_messages_aspn_velocity3d.c',
                    'c/src/datasources_lcm_messages_aspn_directionofmotion2d.c',
                    'c/src/datasources_lcm_messages_aspn_opticalcameraimage.c']

c_args = []
if meson.get_compiler('c').get_id() == 'gcc'
    c_args = ['-Warray-bounds=0', '-Wno-stringop-overread']
endif

aspn_lcm_lib = static_library('aspn-lcm',
                                sources: aspn_lcm_srcs,
                                include_directories: aspn_lcm_inc,
                                dependencies: lcm_dep,
                                c_args: c_args)
aspn_lcm_dep = declare_dependency(include_directories: aspn_lcm_inc,
                                    link_with: aspn_lcm_lib)
aspn_cpp_lcm_dep = declare_dependency(include_directories: aspn_cpp_lcm_inc,
                                        dependencies: [lcm_dep])

meson.override_dependency('aspn-lcm', aspn_lcm_dep)
meson.override_dependency('aspn-cpp-lcm', aspn_cpp_lcm_dep)

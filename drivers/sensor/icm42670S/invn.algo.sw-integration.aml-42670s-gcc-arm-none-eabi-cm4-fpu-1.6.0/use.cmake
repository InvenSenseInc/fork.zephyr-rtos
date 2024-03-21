#!
#! @brief Look for library paths in 3rd party and link them in the current target
#! @param[in] _p_in_libAlgo The library to find
#! @param[in] _p_in_lib_path The library path
#! @details
#! When compiling for a embedded target, only the release build of the libraries can be
#! used. Since hosts are mostly used for debugging/testing purpose, this limitation does
#! not apply.
#!
function (look_for_lib _p_in_libAlgo _p_in_lib_path)
	if (CMAKE_CROSSCOMPILING)
		# Only use release library for embedded platforms
		set (___dir___ "release")
		# Find libraries being generated
		find_library(ALGO_INVN_LIBRARY NAMES ${_p_in_libAlgo} HINTS ${_p_in_lib_path}/${___dir___}/lib)
		if ("${ALGO_INVN_LIBRARY}" STREQUAL "ALGO_INVN_LIBRARY-NOTFOUND")
			mbs_msg_fatal("${_p_in_libAlgo} library could not be found")
		endif()
		# Link with libraries which have been found
		mbs_link_libraries(${ALGO_INVN_LIBRARY})
	else()
		mbs_msg_fatal("aml_use_lib CMAKE_CROSSCOMPILING false, no implementation in use.cmake")
	endif()
	unset (ALGO_INVN_LIBRARY CACHE)
	unset (___dir___)
endfunction()

function (aml_use_lib_40608)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_40608 ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608})
endfunction()

function (aml_use_lib_40608_eval)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_40608_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608_EVAL_PATH})
endfunction()

function (aml_use_lib_open)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_OPEN ${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN_PATH})
endfunction()

function (aml_use_lib_open_eval)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_OPEN_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN-EVAL_PATH})
endfunction()

function (aml_use_lib_40627)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_40627 ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627_PATH})
endfunction()

function (aml_use_lib_40627-eval)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_40627_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627-EVAL_PATH})
endfunction()

function (aml_use_lib_42670p)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_42670P ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670P_PATH})
endfunction()

function (aml_use_lib_42670t)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_42670T ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670T_PATH})
endfunction()

function (aml_use_lib_42670s)
	aml_include_dirs()
	look_for_lib(InvnAlgoAML_42670S ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670S_PATH})
endfunction()

#!
#! @brief Add algorithm libraries include directories to the current target
#!
function (aml_40608_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608_PATH}/include/"
	)
endfunction()

function (aml_40608_eval_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608-EVAL_PATH}/include/"
	)
endfunction()

function (aml_open_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN_PATH}/include/"
	)
endfunction()

function (aml_open_eval_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN-EVAL_PATH}/include/"
	)
endfunction()

function (aml_40627_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627_PATH}/include/"
	)
endfunction()

function (aml_40627_eval_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627-EVAL_PATH}/include/"
	)
endfunction()

function (aml_42670p_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670P_PATH}/include/"
	)
endfunction()

function (aml_42670t_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670T_PATH}/include/"
	)
endfunction()

function (aml_42670s_include_dirs)
	mbs_include_dirs_add(
		"${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670S_PATH}/include/"
	)
endfunction()

#!
#! @brief Install library binaries where requested
#! @param[in] _p_in_install_path_ Where to install the libraries, relative to the current target
#! @param[in] _p_in_component_ (optional) Component under which libraries will be installed (default: "Unspecified")
#! @param[in] _p_in_build_type_ (optional) Configuration for which libraries will be installed (default: current build type)
#!
function (install_lib _p_in_install_path_ _p_in_component_ _p_in_build_type_ _p_in_libAlgo _p_in_lib_path)
	string (TOLOWER ${_p_in_build_type_} ___build___)
	if (CMAKE_CROSSCOMPILING)
		# Only use release library for embedded platforms
		set (___build___ "release")
		# Find generated libraries
		find_library(ALGO_INVN_LIBRARY NAMES ${_p_in_libAlgo} HINTS ${_p_in_lib_path}/${___build___}/lib)
		if ("${ALGO_INVN_LIBRARY}" STREQUAL "ALGO_INVN_LIBRARY-NOTFOUND")
			mbs_msg_fatal("${_p_in_libAlgo} library could not be found")
		endif()

		install(FILES ${ALGO_INVN_LIBRARY}
			DESTINATION ${_p_in_install_path_}
			CONFIGURATIONS ${___build___}
			COMPONENT ${_p_in_component_})

	else()
		# Find shared library being generated
		find_library(ALGO_INVN_LIBRARY NAMES ${_p_in_libAlgo} HINTS ${_p_in_lib_path}/${___build___}/bin
			${_p_in_lib_path}/${___build___}/lib)
		if ("${ALGO_INVN_LIBRARY}" STREQUAL "ALGO_INVN_LIBRARY-NOTFOUND")
			mbs_msg_fatal("${_p_in_libAlgo} library could not be found")
		endif()
		# Sanity check that what has been found matches expectations
		if (${ALGO_INVN_LIBRARY} MATCHES "${___build___}/lib")
			set (_final_dir_ "lib")
		elseif (${ALGO_INVN_LIBRARY} MATCHES "${___build___}/bin")
			set (_final_dir_ "bin")
		else()
			# Impossible combination, exit now
			return()
		endif()
		install(FILES ${ALGO_INVN_LIBRARY}
			DESTINATION ${_p_in_install_path_}/${___build___}/${_final_dir_}
			CONFIGURATIONS ${___build___}
			COMPONENT ${_p_in_component_})
		# Some host environment are also generating a static library for the build,
		# therefore the dynamic library might not be the one to be install by the 
		# below rule. Make sure the dynamic library is always installed.
		mbs_runtime_dependencies_install_for(${___build___} TARGET ${_p_in_libAlgo} "./${___build___}/bin" "")
		unset (_final_dir_)
		unset (ALGO_INVN_LIBRARY CACHE)
	endif()
	unset (ALGO_INVN_LIBRARY CACHE)
	unset (___build___)
endfunction()

function (aml_40608_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_40608 ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608_PATH})
endfunction()

function (aml_40608_eval_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_40608_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40608-EVAL_PATH})
endfunction()

function (aml_40627_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_40627 ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627_PATH})
endfunction()

function (aml_40627_eval_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_40627_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-40627-EVAL_PATH})
endfunction()

function (aml_42670p_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_42670P ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670P_PATH})
endfunction()

function (aml_42670t_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_42670T ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670T_PATH})
endfunction()

function (aml_42670s_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_42670S ${MBS_INVN_ALGO_SW-INTEGRATION_AML-42670S_PATH})
endfunction()

function (aml_open_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_OPEN ${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN_PATH})
endfunction()

function (aml_open_eval_install_lib _p_in_install_path_)
	if (${ARGC} EQUAL 1)
		set(_p_in_component_ "Unspecified")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	elseif(${ARGC} EQUAL 2)
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${CMAKE_BUILD_TYPE}")
	else()
		set(_p_in_component_ "${ARGV1}")
		set(_p_in_build_type_ "${ARGV2}")
	endif()
	install_lib(_p_in_install_path_ _p_in_component_ _p_in_build_type_ InvnAlgoAML_OPEN_EVAL ${MBS_INVN_ALGO_SW-INTEGRATION_AML-OPEN-EVAL_PATH})
endfunction()

#!
#! @brief Install library headers where requested
#! @param[in] _p_in_component_ Which component should install these headers ("Unspecified" when unknown)
#!
function (aml_install_headers _p_in_component_)
	mbs_environment_current_build_types_get(_build_types_)
	foreach(_build_type_ ${_build_types_})
		string (TOLOWER ${_build_type_} ___build___)
		if (CMAKE_CROSSCOMPILING)
			# Only use release library for embedded platforms
			set (___build___ "release")
		endif()
		install(FILES ${MBS_INVN_ALGO_SW-INTEGRATION_AML_PATH}/include/invn_algo_aml.h
			DESTINATION sources/Invn/LibAML
			CONFIGURATIONS ${___build___}
			COMPONENT ${_p_in_component_})
		unset (___build___)
	endforeach()
endfunction()

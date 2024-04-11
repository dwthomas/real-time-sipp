project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20', 'b_ndebug=if-release'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams', 'timer'])
executable('atsipp',
           ['main.cpp', 'randomsippgraph.cpp', 'segment.cpp','sipp.cpp', 'augmentedsipp.cpp', 'rtasipp.cpp', 'plrtosipphonly.cpp', 'hybrid.cpp', 'plrtosipp.cpp'],
           install : true,
           dependencies : [boost_dep],
           cpp_args: '-DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON'
           )

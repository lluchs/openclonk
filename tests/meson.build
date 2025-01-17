gtest = dependency('gtest', required: get_option('tests'))
gmock = dependency('gmock', required: get_option('tests'))

if gtest.found() and gmock.found()
  test_inc = include_directories('..')
  test_sources = [
    'TestLog.cpp',
    'main.cpp',
  ]
  c4script_test_sources = [
    '../src/script/C4ScriptStandaloneStubs.cpp',
    '../src/script/C4ScriptStandalone.cpp',
  ]

  std_mesh_math = executable(
    'StdMeshMath',
    test_sources,
    '../src/lib/StdMeshMath.cpp',
    '../src/lib/StdMeshMath.h',
    'math/StdMeshVectorTest.cpp',
    'math/StdMeshQuaternionTest.cpp',
    dependencies: [allinc, gtest, gmock],
    include_directories: test_inc,
  )
  test('StdMeshMath', std_mesh_math, protocol: 'gtest')

  aul_test = executable(
    'aul_test',
    test_sources,
    c4script_test_sources,
    'aul/AulTest.cpp',
    'aul/AulMathTest.cpp',
    'aul/AulPredefinedFunctionTest.cpp',
    'aul/AulDeathTest.cpp',
    'aul/AulDiagnosticsTest.cpp',
    'aul/AulSyntaxTest.cpp',
    '../src/lib/StdMeshMath.cpp',
    '../src/lib/StdMeshMath.h',
    'math/StdMeshVectorTest.cpp',
    'math/StdMeshQuaternionTest.cpp',
    dependencies: [allinc, libmiscdep, libc4scriptdep, gtest, gmock],
    include_directories: test_inc,
  )
  test('aul_test', aul_test, protocol: 'gtest')

  tests = executable(
    'tests',
    test_sources,
    c4script_test_sources,
    'C4NetIOTest.cpp',
    'C4StringTableTest.cpp',
    'C4ValueTest.cpp',
    'DirectExecTest.cpp',
    'StdFileTest.cpp',
    'UnicodeHandlingTest.cpp',
    dependencies: [allinc, libmiscdep, libc4scriptdep, gtest, gmock],
    include_directories: test_inc,
  )
  test('tests', tests, protocol: 'gtest')
else
  test('gtest not installed!', find_program('python3'), args: ['-c', 'exit(1)'])
endif

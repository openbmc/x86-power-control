gtest = dependency('gtest', disabler: true, required: get_option('tests'))
gtest_main = dependency(
    'gtest_main',
    disabler: true,
    required: get_option('tests'),
)
gmock = dependency('gmock', disabler: true, required: get_option('tests'))

test(
    'test_config_parser',
    executable(
        'test_config_parser',
        'test_config_parser.cpp',
        '../src/config_parser.cpp',
        include_directories: include_directories('../src'),
        dependencies: [gtest, gtest_main, gmock, deps],
    ),
)

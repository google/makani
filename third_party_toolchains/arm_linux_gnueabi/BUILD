package(default_visibility = ['//visibility:public'])

filegroup(
  name = 'gcc',
  srcs = [
    'usr/bin/arm-linux-gnueabi-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    'usr/bin/arm-linux-gnueabi-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    'usr/bin/arm-linux-gnueabi-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    'usr/bin/arm-linux-gnueabi-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    'usr/bin/arm-linux-gnueabi-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    'usr/bin/arm-linux-gnueabi-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    'usr/bin/arm-linux-gnueabi-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    'usr/bin/arm-linux-gnueabi-as',
  ],
)

filegroup(
  name = 'everything',
  srcs = glob([
    '**',
  ]),
)

filegroup(
  name = 'compiler_components',
  srcs = [
    ':gcc',
    ':ar',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
    ':as',
  ],
)
#! /bin/bash

cd qemu-2.2.0_code
./configure --prefix=../qemu-2.2.0
make
make install


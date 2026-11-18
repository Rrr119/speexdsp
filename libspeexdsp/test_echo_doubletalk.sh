#! /bin/sh
# Functional check for the double-talk aware echo canceller:
# generates deterministic synthetic test audio, runs it through the AEC
# (with and without the residual echo suppression preprocessor) and checks
# ERLE, near-end speech preservation and double-talk detector accuracy.

set -e

if test -z "$srcdir"; then
   srcdir=`dirname "$0"`
fi

testdir=testecho_dtd.data
rm -rf "$testdir"
mkdir -p "$testdir"

./testecho_dtd_gen "$testdir"
./testecho_dtd "$testdir"

status=$?
rm -rf "$testdir"
exit $status

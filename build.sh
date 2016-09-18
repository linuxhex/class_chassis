#!/bin/bash

dependencies=("autoscrubber_services" "gs" "gslib")

function help() {
    echo "Usage: ./build.sh <command>"
    echo ""
    echo "Available commands:"
    echo "  deps          Fetch dependencies of service_robot_device"
    echo "  test          Build service_robot_device for test environment."
    echo "  release       Build service_robot_device for release environment."
    echo "  fetch_commit  Generate version.yml that contains commit information of dependencies."
}

function deps() {
    rm -rf autoscrubber_services && git clone git@git.gs-robot.com:autoscrubber/autoscrubber_services.git
    rm -rf gs && git clone git@git.gs-robot.com:chenkan/gs.git
    rm -rf gslib && git clone git@git.gs-robot.com:chenkan/gslib.git
}

function fetch_commit() {
    echo "wc_chassis:" > version.yml
    echo "  url: "`git config --get remote.origin.url` >> version.yml
    echo "  commit: ***this commit***" >> version.yml
    echo "  deps:" >> version.yml
    for repo in ${dependencies[@]}
    do
      echo "  - url: "`cd $repo && git config --get remote.origin.url` >> version.yml
      echo "    commit: "`cd $repo && git log -n 1 --pretty=format:"%H"` >> version.yml
    done
}

function test() {
    bazel build //:wc_chassis --cpu=k8 --host_cpu=k8 -c dbg --curses=no
    rm output -rf && mkdir output
    cp bazel-bin/wc_chassis output
    sudo strip output/wc_chassis
}

function release() {
    bazel build //:wc_chassis --cpu=k8 --host_cpu=k8 -c opt --copt=-g0 --copt=-O3 --copt=-s --strip=always --curses=no
    rm output -rf && mkdir output
    cp bazel-bin/wc_chassis output
    sudo strip output/wc_chassis
}

function rel_with_deb() {
    bazel build //:wc_chassis --cpu=k8 --host_cpu=k8 -c dbg --copt=-g --copt=-O3 --curses=no
    rm output -rf && mkdir output
    cp bazel-bin/wc_chassis output
}

$@

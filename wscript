# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('parrot', ['internet'])
    module.includes = '.'
    module.source = [
        'model/parrot-routing-protocol.cc',
        'model/parrot-packet.cc',
        'model/parrot-rtable.cc',
        'model/brain.cc',
        'model/wings.cc',
        'model/chirp.cc',
        'helper/parrot-helper.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'parrot'
    headers.source = [
        'model/parrot-routing-protocol.h',
        'model/parrot-packet.h',
        'model/parrot-rtable.h',
        'model/pce.h',
        'model/pdc.h',
        'helper/parrot-helper.h',
        ]


    bld.ns3_python_bindings()


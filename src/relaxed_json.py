# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 09:28:02 2021

@author: Timothe
"""
from numba import jit
import re
import json
from parsec import (
    sepBy,
    regex,
    string,
    generate,
    many
)

whitespace = regex(r'\s*', re.MULTILINE)

lexeme = lambda p: p << whitespace

lbrace = lexeme(string('{'))
rbrace = lexeme(string('}'))
lbrack = lexeme(string('['))
rbrack = lexeme(string(']'))
colon = lexeme(string(':'))
comma = lexeme(string(','))
true = lexeme(string('true')).result(True)
false = lexeme(string('false')).result(False)
null = lexeme(string('null')).result(None)
quote = string('"') | string("'")


def number():
    return lexeme(
        regex(r'-?(0|[1-9][0-9]*)([.][0-9]+)?([eE][+-]?[0-9]+)?')
    ).parsecmap(float)


def charseq():


    def string_part():
        return regex(r'[^"\'\\]+')


    def string_esc():
        return string('\\') >> (
            string('\\')
            | string('/')
            | string('b').result('\b')
            | string('f').result('\f')
            | string('n').result('\n')
            | string('r').result('\r')
            | string('t').result('\t')
            | regex(r'u[0-9a-fA-F]{4}').parsecmap(lambda s: chr(int(s[1:], 16)))
            | quote
        )
    return string_part() | string_esc()


class StopGenerator(StopIteration):

    def __init__(self, value):
        self.value = value

@lexeme
@generate

def quoted():
    yield quote
    body = yield many(charseq())
    yield quote
    raise StopGenerator(''.join(body))

@generate

def array():
    yield lbrack
    elements = yield sepBy(value, comma)
    yield rbrack
    raise StopGenerator(elements)


@generate

def object_pair():
    key = yield regex(r'[a-zA-Z][a-zA-Z0-9]*') | quoted
    yield colon
    val = yield value
    raise StopGenerator((key, val))

@generate

def json_object():
    yield lbrace
    pairs = yield sepBy(object_pair, comma)
    yield rbrace
    raise StopGenerator(dict(pairs))

value = quoted | number() | json_object | array | true | false | null
rjson = whitespace >> json_object
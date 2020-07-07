# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Parser for Pack2 definition files."""

import re

from makani.lib.python.pack2 import metadata
from ply import lex
from ply import yacc

# PLY's token and parser rule naming conflicts with our style.
# pylint: disable=invalid-name


class Formatter(object):
  """Pack2 code formatter.

  This class is fed tokens from the lexer and produces a conically formatted
  version of the code.
  """

  # Maps each block type to its line terminating character.
  _new_line_char_map = {
      'bitfield8': ',',
      'bitfield16': ',',
      'bitfield32': ',',
      'enum8': ',',
      'enum16': ',',
      'enum32': ',',
      'header': ';',
      'param': ';',
      'scaled8': ',',
      'scaled16': ',',
      'scaled32': ',',
      'specialize': ';',
      'struct': ';',
  }

  # Tokens that eat whitespace after them.
  _whitespace_eaters = set([
      '[',
      '(',
  ])

  def __init__(self):
    self.preamble = ''
    self.formatted = ''
    self.first_line = True
    self.new_line = ''
    self.block_level = 0
    self.indent = '  '
    self.new_line_char = None
    self.eat_whitespace = False
    self.extra_new_line = False
    self.prev_token = None
    self.eat_new_line = False
    self.ignore_line = False

  def _NextNewLine(self, token):
    """Return the nominal line break for the next statement."""
    if self.block_level == 0 and token == ';':
      self.eat_new_line = True
      return '\n\n'
    if token == '}':
      self.eat_new_line = True
      return '\n\n'
    elif token == '{':
      self.eat_new_line = True
      return '\n'
    elif token == self.new_line_char:
      return '\n'
    else:
      return None

  def _ExtraNewLineAllowed(self, token):
    """Determine if an extra new line is allowed.

    Args:
      token: The token currently being added.

    Returns:
      True if an extra new line is allowed before the current statement.
    """
    if self.block_level < 1:
      return False

    if token == '}':
      return False

    if self.eat_new_line:
      return False

    return True

  def _BlockIndent(self):
    indent = ''
    for _ in range(0, self.block_level):
      indent += self.indent
    return indent

  def _NewLine(self, token):
    """Calculate line break.

    Calculates the appropriate line break sequence to proceed the current
    token being added.

    Args:
      token: The token currently being added.

    Returns:
      A string containing the appropriate line break sequence.
    """

    if self.extra_new_line and self._ExtraNewLineAllowed(token):
      # Single blank lines are allowed within blocks to allow for logical
      # grouping of fields/values.
      new_line = '\n\n'
    else:
      new_line = self.new_line
    self.extra_new_line = False
    self.eat_new_line = False

    return new_line + self._BlockIndent()

  def AddToken(self, token, whitespace):
    """Add a token to the formatter.

    Args:
      token: The token to add.
      whitespace: The nominal whitespace to add before the token.
    """

    # Ignore include lines.  These will be prepended later in alphabetical
    # order.
    if not self.prev_token or self.prev_token == '\n':
      if token == 'include':
        self.ignore_line = True

    if self.ignore_line:
      return

    if self.new_line:
      self.formatted += self._NewLine(token)
    elif not self.first_line and not self.eat_whitespace:
      self.formatted += whitespace

    self.formatted += str(token)

    self.new_line = self._NextNewLine(token)
    self.first_line = False
    self.eat_whitespace = token in self._whitespace_eaters

    if token in self._new_line_char_map:
      self.new_line_char = self._new_line_char_map[token]

    self.prev_token = token

  def AddComment(self, comment):
    """Add comment to the formatter."""
    # Special case comments at the top of files as they are allowed to come
    # before include directive.
    if self.first_line:
      self.preamble += '// ' + str(comment) + '\n'
      return

    if self.prev_token == '\n' or self.first_line:
      # A comment following a new line should be on it's own line.
      self.formatted += self._NewLine('')
    else:
      # Otherwise it should be exactly two spaces after the end of line.
      self.formatted += '  '

    self.formatted += '// ' + str(comment)
    self.new_line = '\n'

  def ProcessNewLine(self, count):
    self.prev_token = '\n'
    self.extra_new_line = count > 1
    self.ignore_line = False

  def EnterBlock(self):
    self.block_level += 1

  def ExitBlock(self):
    self.block_level -= 1


class ParseError(Exception):

  def __init__(self, value, errors):
    super(self.__class__, self).__init__(value)
    self.errors = errors
    self.value = value

  def __str__(self):
    string = self.value + '\n'
    for e in self.errors:
      string += e
    return string


class Lexer(object):
  """Lexer for Pack2 definition files."""

  def __init__(self, error_func):
    self.error_func = error_func
    self.formatter = Formatter()

  def Build(self, **kwargs):
    # Building the lexer is separate from __init__() because the PLY
    # docs warn against calling lex() from __init__
    self.lexer = lex.lex(object=self, **kwargs)

  keywords = [
      'BITFIELD8',
      'BITFIELD16',
      'BITFIELD32',
      'ENUM8',
      'ENUM16',
      'ENUM32',
      'HEADER',
      'INCLUDE',
      'PARAM',
      'SCALED8',
      'SCALED16',
      'SCALED32',
      'SPECIALIZE',
      'STRING',
      'STRUCT',
  ]
  keyword_map = {keyword.lower(): keyword for keyword in keywords}

  tokens = keywords + [
      'ID',

      'COLON',
      'COMMA',
      'EQUAL',
      'SEMICOLON',

      'LCURLY',
      'RCURLY',
      'LPAREN',
      'RPAREN',
      'LSQUARE',
      'RSQUARE',

      'FLOAT_LITERAL',
      'HEX_LITERAL',
      'BIN_LITERAL',
      'NEG_DEC_LITERAL',
      'DEC_LITERAL',
      'STRING_LITERAL',
  ]

  # Ignored characters
  t_ignore = ' \t'

  # Tokens
  #
  # PLY makes use of docstrings in token functions to specify the token regex.
  # Furthermore it uses raw strings because, according the manual, "they are the
  # most convenient way to write regular expression strings."
  #
  # pylint: disable=g-docstring-quotes,g-short-docstring-punctuation

  # The order of the functions here reflects the order in which the
  # lexer matches tokens.
  def t_FLOAT_LITERAL(self, t):
    r'[-+]?[0-9]*\.[0-9]+([eE][-+]?[0-9]+)?'
    self.formatter.AddToken(t.value, ' ')
    t.value = float(t.value)
    return t

  def t_HEX_LITERAL(self, t):
    r'0x[0-9A-Fa-f]+'
    self.formatter.AddToken(t.value, ' ')
    t.value = int(t.value[2:], 16)
    return t

  def t_BIN_LITERAL(self, t):
    r'0b[01]+'
    self.formatter.AddToken(t.value, ' ')
    t.value = int(t.value[2:], 2)
    return t

  def t_NEG_DEC_LITERAL(self, t):
    r'-(0|[1-9][0-9]*)'
    self.formatter.AddToken(t.value, ' ')
    t.value = int(t.value, 10)
    return t

  def t_DEC_LITERAL(self, t):
    r'\+?0|[1-9][0-9]*'
    self.formatter.AddToken(t.value, ' ')
    t.value = int(t.value, 10)
    return t

  def t_STRING_LITERAL(self, t):
    r'"[^"]*"'
    self.formatter.AddToken(t.value, ' ')
    t.value = t.value[1:-1]  # Remove quotes.
    return t

  def t_ID(self, t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    self.formatter.AddToken(t.value, ' ')
    t.type = self.keyword_map.get(t.value, 'ID')
    return t

  def t_comment(self, t):
    r'//[ \t]*(?P<comment_text>.*)'
    self.formatter.AddComment(t.lexer.lexmatch.group('comment_text'))

  def t_newline(self, t):
    r'\n+'
    self.formatter.ProcessNewLine(t.value.count('\n'))
    t.lexer.lineno += t.value.count('\n')

  def t_COLON(self, t):
    r':'  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    return t

  def t_COMMA(self, t):
    r','  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    return t

  def t_EQUAL(self, t):
    r'='  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, ' ')
    return t

  def t_SEMICOLON(self, t):
    r';'  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    return t

  def t_LCURLY(self, t):
    r'\{'  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, ' ')
    self.formatter.EnterBlock()
    return t

  def t_RCURLY(self, t):
    r'\}'  # pylint: disable=invalid-name
    self.formatter.ExitBlock()
    self.formatter.AddToken(t.value, '')
    return t

  def t_LPAREN(self, t):
    r'\('  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    self.formatter.EnterBlock()
    return t

  def t_RPAREN(self, t):
    r'\)'  # pylint: disable=invalid-name
    self.formatter.ExitBlock()
    self.formatter.AddToken(t.value, '')
    return t

  def t_LSQUARE(self, t):
    r'\['  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    return t

  def t_RSQUARE(self, t):
    r'\]'  # pylint: disable=invalid-name
    self.formatter.AddToken(t.value, '')
    return t

  def t_error(self, t):
    self.error_func('%d: Illegal character \'%s\'' % (t.lineno, t.value[0]))
    t.lexer.skip(1)

  # pylint: enable=g-docstring-quotes,g-short-docstring-punctuation


class _DefaultFileLoader(object):

  def __init__(self):
    pass

  def ReadFile(self, file_name):
    with open(file_name, 'r') as f:
      contents = f.read()

    return contents


class Parser(object):
  """Parser for Pack2 definition files."""

  def __init__(self, file_loader=None, loaded_files=None):
    self.lexer = Lexer(error_func=self._RecordError)
    self.lexer.Build()
    self.tokens = self.lexer.tokens

    if loaded_files:
      self.loaded_files = loaded_files
    else:
      self.loaded_files = set()

    if file_loader is None:
      self.file_loader = _DefaultFileLoader()
    else:
      self.file_loader = file_loader

    self.include_re = re.compile(r'(.*)\.p2$')
    # TODO: Investigate generating tables at build time and
    # packaging them with the library.
    self.parser = yacc.yacc(module=self, debug=False, write_tables=False)

  def Parse(self, string):
    """Parse a Pack2 definition string."""

    self.valid = True
    self.metadata = metadata.Metadata()
    self.errors = []
    try:
      self.parser.parse(string, tracking=True)
    except IndexError as e:
      # Due to a bug in PLY, an index error is caused if we raise a syntax
      # error.  If we've previously raised a syntax error, ignore it so that
      # we can raise a ParseError instead.
      if self.valid:
        raise e

    if not self.valid:
      raise ParseError('Parse Error', self.errors)

    return self.metadata

  def GetFormattedSource(self):
    preamble = self.lexer.formatter.preamble
    if self.metadata.includes:
      if preamble:
        preamble += '\n'
      for inc in sorted(self.metadata.includes):
        preamble += ('include "%s.p2";\n' % inc)
      preamble += '\n'

    return preamble + self.lexer.formatter.formatted + '\n'

  def _RecordError(self, string):
    self.valid = False
    self.errors.append(string)

  def _RaiseError(self, string):
    self._RecordError(string)
    raise SyntaxError(string)

  def HandleWidthType(self, base_name, p):
    """Common handing for types that have 8, 16, and 32 bit widths.

    Grammar for type should be of the follow:
      type_def : type_keyword ID LCURLY type_body RCURLY

    Args:
      base_name: The type's base name (eg. 'enum', 'bitfield', or 'scaled'.)
      p: The PLY parser arguments from the production rule.

    Returns:
      A dict containing 'type', 'name', 'body', and 'width'.
    """
    info = {
        'type': p[1],
        'name': p[2],
        'body': p[4],
    }

    if info['type'] == base_name + '8':
      info['width'] = 1
    elif info['type'] == base_name + '16':
      info['width'] = 2
    elif info['type'] == base_name + '32':
      info['width'] = 4
    else:
      self._RaiseError('%d: invalid %s type %s.\n'
                       % (p.lineno(1), base_name, info['type']))
    return info

  def ResolveType(self, type_name, lineno=-1):
    if type_name not in self.metadata.type_map:
      self._RaiseError('%d: Type \'%s\' unknown.\n' % (lineno, type_name))
      raise SyntaxError
    return self.metadata.type_map[type_name]

  # PLY makes use of docstrings in production function to specify the grammar.
  # These do not conform to the google style for doc strings.
  #
  # pylint: disable=g-short-docstring-punctuation
  # pylint: disable=g-doc-args
  # pylint: disable=g-no-space-after-docstring-summary
  def p_file(self, p):
    """file : bitfield_def file
            | enum_def file
            | header_def file
            | include_def file
            | param_def file
            | scaled_def file
            | specialize_def file
            | struct_def file
            |
    """

  def p_include_def(self, p):
    """include_def : INCLUDE STRING_LITERAL SEMICOLON"""
    file_name = p[2]

    match = self.include_re.match(file_name)
    if not match:
      self._RaiseError('%d: %s is not named like a p2 file.' % (p.lineno(2),
                                                                file_name))
    path = match.group(1)

    if file_name in self.loaded_files:
      return
    self.loaded_files.add(file_name)

    contents = self.file_loader.ReadFile(file_name)
    parser = Parser(file_loader=self.file_loader,
                    loaded_files=self.loaded_files)
    meta = parser.Parse(contents)

    try:
      self.metadata.AddInclude(path, meta)
    except ValueError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_struct_def(self, p):
    """struct_def : STRUCT ID LCURLY struct_body RCURLY"""
    name = p[2]
    body = p[4]

    try:
      self.metadata.AddType(metadata.StructType(name, body))
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_enum_def(self, p):
    """enum_def : enum_keyword ID LCURLY enum_body RCURLY"""
    try:
      info = self.HandleWidthType('enum', p)
      enum = metadata.EnumType(info['name'], info['width'], info['body'])
      self.metadata.AddType(enum)
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))
    except ValueError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_enum8_keyword(self, p):
    """enum_keyword : ENUM8
                    | ENUM16
                    | ENUM32
    """
    p[0] = p[1]

  def p_bitfield_def(self, p):
    """bitfield_def : bitfield_keyword ID LCURLY bitfield_body RCURLY"""
    try:
      info = self.HandleWidthType('bitfield', p)
      bitfield = metadata.BitfieldType(info['name'],
                                       info['width'],
                                       info['body'])
      self.metadata.AddType(bitfield)
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))
    except ValueError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_bitfield8_keyword(self, p):
    """bitfield_keyword : BITFIELD8
                        | BITFIELD16
                        | BITFIELD32
    """
    p[0] = p[1]

  def p_scaled_def(self, p):
    """scaled_def : scaled_keyword ID LCURLY scaled_body RCURLY"""
    try:
      info = self.HandleWidthType('scaled', p)

      if 'scale' not in info['body']:
        self._RaiseError('%d: Scaled type %s does not contain scale property.'
                         %(p.lineno(2), info['name']))
      if 'offset' not in info['body']:
        self._RaiseError('%d: Scaled type %s does not contain offset property.'
                         %(p.lineno(2), info['name']))

      scale = info['body']['scale']
      offset = info['body']['offset']
      scaled = metadata.ScaledType(info['name'], info['width'],
                                   offset=offset, scale=scale)
      self.metadata.AddType(scaled)
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))
    except ValueError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_scaled8_keyword(self, p):
    """scaled_keyword : SCALED8
                      | SCALED16
                      | SCALED32
    """
    p[0] = p[1]

  def p_param_def(self, p):
    """param_def : PARAM ID LCURLY struct_body RCURLY"""
    name = p[2]
    body = p[4]

    try:
      param = metadata.Param(name, body)
      self.metadata.AddType(param)
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_header_def(self, p):
    """header_def : HEADER ID LCURLY struct_body RCURLY"""
    name = p[2]
    body = p[4]

    try:
      header = metadata.Header(name, body)
      self.metadata.AddType(header)
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  # pylint: disable=line-too-long
  def p_speclialize_def(self, p):
    """specialize_def : SPECIALIZE LPAREN ID RPAREN ID LCURLY struct_body RCURLY"""
    # pylint: enable=line-too-long
    parent_name = p[3]
    name = p[5]
    body = p[7]

    if parent_name not in self.metadata.type_map:
      self._RaiseError('%d: Unknown parent type %s.\n'
                       % (p.lineno(2), parent_name))
    parent_type = self.metadata.type_map[parent_name]
    try:
      new_type = parent_type.Specialize(name, body)
      self.metadata.AddType(new_type)

    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))
    except ValueError as e:
      self._RaiseError('%d: %s\n' % (p.lineno(2), e))

  def p_struct_body(self, p):
    """struct_body : struct_body field_def
                   | field_def
    """
    try:
      if len(p) == 2:
        line = p.lineno(1)
        body = metadata.StructBody()
        body.AddField(p[1])
      elif len(p) == 3:
        line = p.lineno(2)
        body = p[1]
        body.AddField(p[2])
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (line, e))
    p[0] = body

  def p_field_def(self, p):
    """field_def : ID ID SEMICOLON"""
    type_name = p[1]
    name = p[2]

    field_type = self.ResolveType(type_name, p.lineno(1))
    p[0] = metadata.StructField(field_type, name)

  def p_string_field_def(self, p):
    """field_def : STRING LSQUARE unsigned_literal RSQUARE ID SEMICOLON"""
    length = p[3]
    name = p[5]

    type_obj = metadata.StringType(length)
    p[0] = metadata.StructField(type_obj, name)

  def p_array_field_def(self, p):
    """field_def : ID ID LSQUARE unsigned_literal RSQUARE SEMICOLON"""
    type_name = p[1]
    name = p[2]
    extent = p[4]

    field_type = self.ResolveType(type_name, p.lineno(1))
    p[0] = metadata.StructField(field_type, name, extent)

  def p_enum_body(self, p):
    """enum_body : enum_body enum_value
                 | enum_value
    """
    try:
      if len(p) == 2:
        line = p.lineno(1)
        value = p[1]
        body = metadata.EnumBody()
      elif len(p) == 3:
        line = p.lineno(2)
        value = p[2]
        body = p[1]

      body.AddValue(value[0], value[1])
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (line, e))
    p[0] = body

  def p_enum_value(self, p):
    """enum_value : ID EQUAL signed_literal COMMA"""
    p[0] = (p[1], p[3])

  def p_bitfield_body(self, p):
    """bitfield_body : bitfield_body bitfield_value
                     | bitfield_value
    """
    try:
      if len(p) == 2:
        line = p.lineno(1)
        value = p[1]
        body = metadata.BitfieldBody()
      elif len(p) == 3:
        line = p.lineno(2)
        value = p[2]
        body = p[1]

      body.AddFlag(value[0], value[1])
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (line, e))
    p[0] = body

  def p_scaled_body(self, p):
    """scaled_body : scaled_body scaled_property
                   | scaled_property
    """
    try:
      if len(p) == 2:
        line = p.lineno(1)
        value = p[1]
        body = {}
      elif len(p) == 3:
        line = p.lineno(2)
        value = p[2]
        body = p[1]

      if value[0] in body:
        self._RaiseError('%d: Scaled property %s repeated.' % (line, value[0]))

      body[value[0]] = value[1]
    except SyntaxError as e:
      self._RaiseError('%d: %s\n' % (line, e))
    p[0] = body

  def p_scaled_property(self, p):
    """scaled_property : ID EQUAL FLOAT_LITERAL COMMA
                       | ID EQUAL signed_literal COMMA
    """
    name = p[1]
    value = p[3]

    if name != 'scale' and name != 'offset':
      self._RaiseError('%d: Unknown scaled property %s.' % (p.lineno(1), name))

    p[0] = (name, value)

  def p_bitfield_value(self, p):
    """bitfield_value : unsigned_literal COLON ID COMMA"""
    p[0] = (p[3], p[1])

  def p_unsigned_literal(self, p):
    """unsigned_literal : HEX_LITERAL
                        | DEC_LITERAL
                        | BIN_LITERAL
    """
    p[0] = p[1]

  def p_signed_literal(self, p):
    """signed_literal : unsigned_literal
                      | NEG_DEC_LITERAL
    """
    p[0] = p[1]

  def p_error(self, p):
    self.valid = False
    self.errors.append('%d: Syntax error at \'%s\'\n' % (p.lineno, p.value))

  # pylint: enable=g-short-docstring-punctuation
  # pylint: enable=g-no-space-after-docstring-summary
  # pylint: enable=g-no-space-after-docstring-summary
  # pylint: enable=invalid-name

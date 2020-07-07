"
" syntax/pack2.vim: Vim syntax file for pack2 metadata files.
"
" Language: Pack2

if exists("b:current_syntax")
  finish
endif

syn keyword     pack2Keywords       bitfield8 bitfield16 bitfield32
syn keyword     pack2Keywords       enum8 enum16 enum32
syn keyword     pack2Keywords       scaled8 scaled16 scaled32
syn keyword     pack2Keywords       header param struct
syn keyword     pack2Keywords       include specialize

hi def link     pack2Keywords       Keyword

syn keyword     pack2Types          uint8 uint16 uint32
syn keyword     pack2Types          int8 int16 int32
syn keyword     pack2Types          string float date

hi def link     pack2Types          Type

" Comments; their contents.
syn keyword     pack2Todo           contained TODO FIXME XXX BUG
syn cluster     pack2CommentGroup   contains=pack2Todo
syn region      pack2Comment        start="//" end="$" contains=@pack2CommentGroup,@Spell

hi def link     pack2Comment        Comment
hi def link     pack2Todo           Todo

syn match       pack2SpaceError     display excludenl "\s\+$"
hi def link     pack2SpaceError     Error

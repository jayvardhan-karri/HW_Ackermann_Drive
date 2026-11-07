-- Pandoc filter for the LIMO PDFs.
-- 1. Removes hand-written numbers ("3.", "2.1") at the start of headings, so LaTeX
--    numbers every section the same way.
-- 2. Inline code: escapes it for LaTeX and adds a break point after / _ . : = , - | * so
--    long paths and commands wrap instead of running into the margin.
function Header(el)
  local first = el.content[1]
  if first and first.t == 'Str' and first.text:match('^%d+[%.%d]*%.?$') then
    table.remove(el.content, 1)
    if el.content[1] and el.content[1].t == 'Space' then table.remove(el.content, 1) end
  end
  return el
end

local escapes = {
  ['\\'] = '\\textbackslash{}', ['{'] = '\\{', ['}'] = '\\}', ['$'] = '\\$', ['&'] = '\\&',
  ['#'] = '\\#', ['^'] = '\\textasciicircum{}', ['_'] = '\\_', ['%'] = '\\%', ['~'] = '\\textasciitilde{}',
  ['<'] = '\\textless{}', ['>'] = '\\textgreater{}',
}
local breaks = { ['/'] = true, ['_'] = true, ['.'] = true, [':'] = true, ['='] = true, [','] = true,
                 ['-'] = true, ['|'] = true, ['*'] = true }

function Code(el)
  if FORMAT ~= 'latex' then return nil end
  local out = {}
  for _, cp in utf8.codes(el.text) do
    local ch = utf8.char(cp)
    out[#out + 1] = escapes[ch] or ch
    if breaks[ch] then out[#out + 1] = '\\allowbreak{}' end
  end
  return pandoc.RawInline('latex', '\\texttt{' .. table.concat(out) .. '}')
end

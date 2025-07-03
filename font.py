#!/usr/bin/python3

from dataclasses import dataclass
from typing import Callable
import io
import sys

symbol_chars: str = '''
 𜺨𜺫🮂𜴀▘𜴁𜴂𜴃𜴄▝𜴅𜴆𜴇𜴈▀
𜴉𜴊𜴋𜴌🯦𜴍𜴎𜴏𜴐𜴑𜴒𜴓𜴔𜴕𜴖𜴗
𜴘𜴙𜴚𜴛𜴜𜴝𜴞𜴟🯧𜴠𜴡𜴢𜴣𜴤𜴥𜴦
𜴧𜴨𜴩𜴪𜴫𜴬𜴭𜴮𜴯𜴰𜴱𜴲𜴳𜴴𜴵🮅
𜺣𜴶𜴷𜴸𜴹𜴺𜴻𜴼𜴽𜴾𜴿𜵀𜵁𜵂𜵃𜵄
▖𜵅𜵆𜵇𜵈▌𜵉𜵊𜵋𜵌▞𜵍𜵎𜵏𜵐▛
𜵑𜵒𜵓𜵔𜵕𜵖𜵗𜵘𜵙𜵚𜵛𜵜𜵝𜵞𜵟𜵠
𜵡𜵢𜵣𜵤𜵥𜵦𜵧𜵨𜵩𜵪𜵫𜵬𜵭𜵮𜵯𜵰
𜺠𜵱𜵲𜵳𜵴𜵵𜵶𜵷𜵸𜵹𜵺𜵻𜵼𜵽𜵾𜵿
𜶀𜶁𜶂𜶃𜶄𜶅𜶆𜶇𜶈𜶉𜶊𜶋𜶌𜶍𜶎𜶏
▗𜶐𜶑𜶒𜶓▚𜶔𜶕𜶖𜶗▐𜶘𜶙𜶚𜶛▜
𜶜𜶝𜶞𜶟𜶠𜶡𜶢𜶣𜶤𜶥𜶦𜶧𜶨𜶩𜶪𜶫
▂𜶬𜶭𜶮𜶯𜶰𜶱𜶲𜶳𜶴𜶵𜶶𜶷𜶸𜶹𜶺
𜶻𜶼𜶽𜶾𜶿𜷀𜷁𜷂𜷃𜷄𜷅𜷆𜷇𜷈𜷉𜷊
𜷋𜷌𜷍𜷎𜷏𜷐𜷑𜷒𜷓𜷔𜷕𜷖𜷗𜷘𜷙𜷚
▄𜷛𜷜𜷝𜷞▙𜷟𜷠𜷡𜷢▟𜷣▆𜷤𜷥█'''.replace('\n', '')

assert len(symbol_chars) == 256

symbols: list[str] = [s for s in symbol_chars]

def gen_symbols():
    symbols = ['' for _ in range(256)]

    symbols[0] = ' '
    symbols[  1] = '\U0001CEA8'
    symbols[  2] = '\U0001CEAB'
    symbols[  3] = '\U0001FB82'
    symbols[  5] = '\u2598'
    symbols[ 10] = '\u259D'
    symbols[ 15] = '\u2580'
    symbols[ 20] = '\U0001FBE6'
    symbols[ 40] = '\U0001FBE7'
    symbols[ 63] = '\U0001FB85'
    symbols[ 64] = '\U0001CEA3'
    symbols[ 80] = '\u2596'
    symbols[ 85] = '\u258C'
    symbols[ 90] = '\u259E'
    symbols[ 95] = '\u259B'
    symbols[128] = '\U0001CEA0'
    symbols[160] = '\u2597'
    symbols[165] = '\u259A'
    symbols[170] = '\u2590'
    symbols[175] = '\u259C'
    symbols[192] = '\u2582'
    symbols[240] = '\u2584'
    symbols[245] = '\u2599'
    symbols[250] = '\u259F'
    symbols[252] = '\u2586'
    symbols[255] = '\u2588'

    for L in open('NamesList.txt'):
        ll = L.strip().split('\t')
        if len(ll) != 2 or not ll[1].startswith('BLOCK OCTANT-'):
            continue
        c = chr(int(ll[0], 16))
        #print(c)
        octants = ll[1].removeprefix('BLOCK OCTANT-')
        index = sum(1 << (ord(c) - ord('1')) for c in octants)
        assert symbols[index] is None
        symbols[index] = c

    for i in range(0, 256, 16):
        s = ''.join(symbols[i:i+16])
        print(s);
    return symbols

if symbols == None:
    symbols = gen_symbols()

@dataclass
class Character:
    character: str
    description: str
    width: int
    height: int
    rows: list[int]
    def columns(self) -> list[int]:
        data = []
        for x in range(self.width):
            bits = 0
            for y, r in enumerate(self.rows):
                if r & (1 << x):
                    bits |= 1 << y
            data.append(bits)
        return data
    def print_row(self, pre, w, f: Callable[[int],int] = lambda x: x):
        print(pre, end='')
        sep = '['
        for x, n in enumerate(map(f, self.columns())):
            print(f'{sep}{n:>{w}}', end='')
            sep = ','
        if self.character.strip():
            print('], //', self.character)
        else:
            print('],')
    def cycle(self,
              tag: str, width: int, cycle: int, offset: int) -> 'Character':
        description = self.description + tag
        rows = [0 for _ in self.rows]
        bits = 0
        while bits < width:
            start = (bits + offset) % cycle
            num = min(width - bits, self.width - start)
            mask = (1 << num) - 1
            for i, me in enumerate(self.rows):
                rows[i] |= ((me >> start) & mask) << bits
            bits += num
        return Character(description, description, width, self.height, rows)

def fontfile(ff: io.TextIOWrapper) -> dict[str, Character]:
    '''Read my font.txt'''
    characters = {}
    characters[' '] = Character(' ', 'SPACE', 10, 1, [0])

    f = iter(ff)
    n: str|None = next(f)
    while n is not None:
        assert n.startswith('char ')
        item = n.strip('\n').split()
        assert len(item) in (2, 3)
        character = item[1]
        if len(item) == 3:
            description = item[2]
        elif character.isdigit():
            description = 'DIGIT_' + character
        elif character.isalpha():
            description = 'LETTER_' + character
        else:
            description = character
        n = None
        rows = []
        maxbit = 0
        anchor = None
        for L in f:
            if L.startswith('char'):
                n = L
                break
            row = 0
            LL = L.rstrip('\n')
            for bit, c in enumerate(LL):
                if c in '*@':
                    row += 1 << bit
                else:
                    assert c in ' .', LL
            rows.append(row)
            if '@' in LL or '.' in LL:
                anchor = len(rows)
            if len(LL) > maxbit:
                maxbit = len(LL)
        if anchor is None:
            anchor = len(rows)
        # Pad to 10 × n
        for i, r in enumerate(rows):
            rows[i] = r << (10 - maxbit) // 2
        assert anchor <= 13, f'{item} {anchor}'
        assert len(rows) - anchor <= 3, f'{item} {anchor}'
        for _ in range(anchor, 13):
            rows.insert(0, 0)
        while len(rows) < 16:
            rows.append(0)
        assert len(rows) == 16, f'{item} {len(rows)}'
        #assert len(item) == 6
        characters[character] = Character(character, description, 10, 16, rows)
    return characters

def printchars(characters: dict[str, Character]):
    '''Print characters using UTF-8 2×4 blocks.'''
    for character in characters.values():
        rows = list(character.rows)
        while len(rows) % 4 != 0:
            rows.append(0)
        string = ''
        for Y in range(0, len(rows), 4):
            #print(f'Y = {Y}')
            for X in range(0, character.width, 2):
                #print(f'X = {X}')
                bits = 0
                for y in range(4):
                    for x in 0, 1:
                        if rows[Y + y] & (1 << (X + x)) != 0:
                            bits += 1 << (y * 2 + x)
                #print(bits)
                string += symbols[bits]
            string += '\n'
        print(character.description)
        print(string)

def bdffile(path: str) -> dict[str, Character]:
    '''Read a BDF file'''
    f = open(path)
    lines = iter(l.strip() for l in f)
    characters = {}
    font_ascent = 0
    for line in lines:
        if line.startswith('FONT_ASCENT '):
            font_ascent = int(line.split(' ', 1)[1])
        if not line.startswith('STARTCHAR '):
            continue
        name = line.split(' ', 1)[1].strip()
        character = name
        dwidth = 0
        for line in lines:
            if line.startswith('BBX '):
                width, height, xorig, yorig = map(int, line.split()[1:])
            if line.startswith('DWIDTH '):
                dwidth = int(line.split(' ')[1])
            if line.startswith('ENCODING '):
                character = chr(int(line.split(' ')[1]))
            if line == 'BITMAP':
                break
        rows = [0] * max(font_ascent - height - yorig, 0)
        #print(character, width, height, xorig, yorig)
        for _ in range(height):
            data = reversed(next(lines))
            word = ''.join('084C2A6E195D3B7F'[int(w, 16)] for w in data)
            rows.append(int(word, 16) << max(xorig, 0))
        characters[name] = Character(character, name, dwidth, height, rows)
    return characters

def compile(characters: dict[str, Character]):
    '''Convert to binary'''
    #          0123456789abcdef0123456789abcdef01
    charset = ' AVWm.°-0123456789'
    #charset = ' AbCdeVghim+W-.°0123456789:;<=>?'

    wanted = [characters[c] for c in charset]

    for side in 'LEFT', 'RIGHT':
        for tb in 'TOP', 'BOTTOM':
           for phase in range(8):
               wanted.append(characters[f'{side}_{tb}_{phase}'])

    print('#![allow(dead_code, non_upper_case_globals)]')
    print();
    print(f'pub static CHARS: &str = r#"{charset}"#;');
    print();

    seen = set()
    for n, c in enumerate(wanted):
        if not c.description in seen:
            seen.add(c.description)
            print(f'pub const {c.description}: u8 = {n};')
    print();

    count = len(wanted);
    intro = f'pub static FONT10X16: [[[u8; 10]; {count}]; 2] = [['
    for name, f in ('TOP', lambda n: n & 0xff), ('BOT', lambda n: n >> 8):
        print(intro)
        intro = '    ['
        for c in wanted:
            c.print_row('        ', 4, f);
        print('    ],')
    print('];')

def print_list(data: list[list[int]]):
    for c in data:
        pass

def box(characters, width, height):
    '''Surround each character by its bounding box.'''
    for c in characters.values():
        if width is None:
            w = c.width
        else:
            w = width
        full = (1 << w + 2) - 1
        side = (1 << w + 1) + 1
        rows = c.rows
        while len(rows) > height and rows[-1] == 0:
            del rows[-1]
        while len(rows) > height and rows[0] == 0:
            del rows[0]
        while len(rows) < height:
            rows.append(0)
        assert len(rows) == height
        for i in range(len(rows)):
            rows[i] = rows[i] * 2 ^ side
        rows.insert(0, full)
        rows.append(full)
        c.width = max(c.width + 1, w + 2)

#characters = bdffile('/home/mirror/xorg/font-bh-100dpi-1.0.3/lubB14.bdf')
#characters = bdffile('/home/mirror/xorg/font-adobe-75dpi-1.0.3/courR18.bdf')
#characters = bdffile('/home/mirror/xorg/font-adobe-75dpi-1.0.3/helvR18.bdf')
#characters = bdffile('/home/mirror/xorg/font-bitstream-75dpi-1.0.3/term14.bdf')
#characters = bdffile('/home/mirror/xorg/font-bitstream-100dpi-1.0.3/term14.bdf')
#characters = bdffile('/home/mirror/xorg/font-bh-lucidatypewriter-75dpi-1.0.3/lutBS14.bdf')
#characters = bdffile('/home/mirror/xorg/font-bh-lucidatypewriter-75dpi-1.0.3/lutRS14.bdf')
#characters = bdffile('/home/mirror/xorg/font-bh-lucidatypewriter-100dpi-1.0.3/lutRS14.bdf')
#characters = bdffile('/home/mirror/xorg/font-schumacher-misc-1.1.2/clR8x16.bdf')
characters = fontfile(open('font-10x16.txt'))

for side in 'LEFT', 'RIGHT':
    for tb in 'TOP', 'BOTTOM':
        for phase in range(8):
            base = characters[f'{side}_{tb}']
            tag = f'_{phase}'
            offset = phase if side == 'LEFT' else 8 - phase
            c: Character = base.cycle(tag, 10, 8, offset)
            characters[c.character] = c

if len(sys.argv) > 1 and sys.argv[1] == 'compile':
    compile(characters)
    sys.exit(0)

box(characters, 10, 16)
printchars(characters)

# macropad-MLDH
Experimental Macropad

## Command-Line Help

### Commands

- `show`  
  Display the current keymap and encoder assignments.

- `set key <row 0..2> <col 0..2> <macro>`  
  Assign a macro to a key at the specified row and column.

- `set enc <0|1> <cw|ccw> <macro>`  
  Assign a macro to an encoder direction.

- `save`  
  Save the current configuration to non-volatile storage.

- `resetdefaults`  
  Restore the default keymap and encoder configuration.

### Macro Format

Macros can be specified using readable key combinations or raw hex values.

Examples:
- `ctrl+c`
- `win+shift+s`
- `alt+f4`
- `0xMM:0xKK` (modifier + key)
- `0xKK` (key only)

# Plan for Extending Graph Selection in `osgar.replay`

This plan outlines the changes needed to allow selecting specific graph outputs in `osgar.replay` and specifically for the `Matty` platform.

## 1. Modify `osgar/replay.py`

### Goal
Extend the `--draw` command line argument to accept an optional selection string.

### Changes
- Change the definition of `--draw` in `argparse`:
  ```python
  parser.add_argument('--draw', help="draw debug results", nargs='?', const=True)
  ```
- Update the call to `module_instance.draw()` to pass the selection:
  ```python
  if args.draw:
      if args.draw is True:
          module_instance.draw()
      else:
          module_instance.draw(args.draw)
  ```
- (Optional but recommended) Add a check to handle modules that do not yet support arguments in their `draw` method to maintain backward compatibility.

## 2. Update `osgar/platforms/matty.py`

### Goal
Implement the `draw` method to handle selections and provide help.

### Changes
- Modify the `draw` method signature: `def draw(self, selection=None):`.
- Implement selection logic:
  - If `selection` is `'enc'`, call `self.draw_enc()`.
  - If `selection` is `'imu'`, call `self.draw_imu()`.
  - If `selection` is `'joint_angle'` (or default `None`/`True`), call `self.draw_joint_angle()`.
  - If `selection` is `'help'`, list available options.

## 3. Proposal for Listing Options

### Goal
Provide a user-friendly way to discover available graph options for a module.

### Proposal
1. **`help` selection**: Conventionally support `--draw help` for any module that has multiple graphs.
   - When `module.draw('help')` is called, it should print the available options to the console.
2. **Automatic listing in `replay.py`**:
   - If `args.draw` is `'help'`, `osgar/replay.py` should attempt to call `module_instance.draw('help')`.
   - If the module does not support arguments or doesn't provide help, `replay.py` can report that no specific options are defined for this module.
3. **Consistency across modules**: Encouraging other platforms (like `Kloubak`, `Spider`, etc.) to adopt the same `draw(selection)` pattern.

## 4. Verification Plan

- Run `python -m osgar.replay --module matty --draw` to verify default behavior (joint angle).
- Run `python -m osgar.replay --module matty --draw help` to verify option listing.
- Run `python -m osgar.replay --module matty --draw enc` to verify encoder graph.
- Run `python -m osgar.replay --module matty --draw imu` to verify IMU graph.
- Test with another module that doesn't support arguments (e.g., `vesc`) to ensure it doesn't crash (if backward compatibility logic is added).

# Plan for Extending Graph Selection in `osgar.replay`

This plan outlines the changes needed to allow selecting specific graph outputs in `osgar.replay` and specifically for the `Matty` platform.

## 1. Modify `osgar/replay.py`

### Goal
Extend the `--draw` command line argument to accept an optional selection string and ensure robust execution.

### Changes
- Change the definition of `--draw` in `argparse`:
  ```python
  parser.add_argument('--draw', help="draw debug results", nargs='?', const=True)
  ```
- Update the call to `module_instance.draw()` to be safe and pass the selection:
  ```python
  if args.draw:
      draw_func = getattr(module_instance, 'draw', None)
      if draw_func:
          if args.draw is True:
              draw_func()
          else:
              try:
                  draw_func(args.draw)
              except TypeError:
                  # Fallback for modules that don't support arguments yet
                  g_logger.warning(f"Module {args.module} does not support draw arguments.")
                  draw_func()
      else:
          g_logger.warning(f"Module {args.module} does not have a draw() method.")
  ```

## 2. Update `osgar/platforms/matty.py`

### Goal
Implement the `draw` method to handle selections and provide help.

### Changes
- Modify the `draw` method signature: `def draw(self, selection=None):`.
- Implement selection logic:
  - If `selection` is `'enc'`, call `self.draw_enc()`.
  - If `selection` is `'imu'`, call `self.draw_imu()`.
  - If `selection` is `'joint_angle'` (or default `None`/`True`), call `self.draw_joint_angle()`.
  - If `selection` is `'help'`, list available options: `enc`, `imu`, `joint_angle`.

## 3. Proposal for Listing Options

### Goal
Provide a user-friendly way to discover available graph options for a module.

### Proposal
1. **`help` selection**: Conventionally support `--draw help` for any module that has multiple graphs.
   - When `module.draw('help')` is called, it should print the available options to the console.
2. **Standardized Attribute**: Consider adding a `draw_options` attribute to modules to allow automated listing in the future.

## 4. Documentation Update (`doc/deep_dive.md`)

### Goal
Document the `osgar.replay` tool and the `--draw` functionality.

### Changes
- Add a new section "11. Replay and Visualization (`osgar.replay`)" to `doc/deep_dive.md`.
- Explain how to use `--module` and `--draw`.
- Describe the new graph selection syntax and the `--draw help` convention.

## 5. Verification Plan

- Run `python -m osgar.replay --module matty --draw` to verify default behavior (joint angle).
- Run `python -m osgar.replay --module matty --draw help` to verify option listing.
- Run `python -m osgar.replay --module matty --draw enc` to verify encoder graph.
- Run `python -m osgar.replay --module matty --draw imu` to verify IMU graph.
- Test with a module without `draw()` (e.g. `osgar.Node`) to ensure it logs a warning instead of crashing.

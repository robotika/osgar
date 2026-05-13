# Plan for Extending Graph Selection in `osgar.replay`

This plan outlines the changes needed to allow selecting specific graph outputs in `osgar.replay` and specifically for the `Matty` platform.

## 1. Modify `osgar/replay.py`

### Goal
Extend the `--draw` command line argument to accept an optional selection string and provide help via docstrings.

### Changes
- Change the definition of `--draw` in `argparse`:
  ```python
  parser.add_argument('--draw', help="draw debug results", nargs='?', const=True)
  ```
- **Docstring-based Help**: 
  - If `args.draw == 'help'`, look for the `module_instance.draw` method.
  - If it exists, print its docstring (`__doc__`) using `inspect.cleandoc` to ensure clean formatting.
  - Exit immediately without replaying the log.
- **Robust Execution**:
  - Proceed with replay for other selections.
  - Call `draw(selection)` at the end, handling cases where the method doesn't support arguments or is missing.

## 2. Update `osgar/platforms/matty.py`

### Goal
Update the `draw` method to use docstrings for documentation and handle selections.

### Changes
- Update the `draw` method docstring to list available options: `enc`, `imu`, `joint_angle`.
- Implement selection logic (without the internal `help` check):
  - `enc`: calls `self.draw_enc()`
  - `imu`: calls `self.draw_imu()`
  - `joint_angle` (or default): calls `self.draw_joint_angle()`

## 3. Documentation Update (`doc/deep_dive.md`)

### Changes
- Update the description of the `--draw help` convention to explain that it now uses Python docstrings from the `draw()` method.

## 4. Verification Plan

- Run `python -m osgar.replay --module matty --draw help` and verify the docstring is printed correctly.
- Verify specific selections (`enc`, `imu`) still work as expected.

---

## Proposed PR Description

### Summary
This PR extends `osgar.replay` to support selecting specific graph outputs via the `--draw` flag. It introduces a convention where available graph options are documented in the `draw()` method's docstring and displayed to the user via `--draw help`.

### Key Changes
- **`osgar.replay`**: 
    - Updated `--draw` to accept an optional argument.
    - Added a help mechanism that prints the `draw()` method's docstring and exits early.
    - Added robust handling for calling `draw()` across different module types.
- **`Matty` Platform**:
    - Documented available graphs (`enc`, `imu`, `joint_angle`) in the `draw()` docstring.
    - Updated the method to switch between these graphs based on the provided selection.
- **Documentation**:
    - Updated `doc/deep_dive.md` to reflect the new `--draw` selection and docstring-based help convention.

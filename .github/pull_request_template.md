## What this changes

<!-- One or two sentences. -->

## Simulator suites

CI runs only the tests that need no simulator. Anything touching locomotion, navigation,
manipulation or the sensor path has to be checked locally:

```bash
colcon test --packages-select <pkg> --ctest-args -L simulator
```

- [ ] Ran the relevant simulator suites, or this change cannot affect them.
- [ ] No new publisher on a low-level channel that already has one.

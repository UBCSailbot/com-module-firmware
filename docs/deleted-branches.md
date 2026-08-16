# Deleted branches

Branches removed from `origin` on 2026-08-16, kept here so the work stays
recoverable. GitHub does not keep deleted refs indefinitely.

Restore one with:

```sh
git push origin <sha>:refs/heads/<name>
```

## Contained in v0.1.0

Zero unique commits at deletion. Nothing to recover.

| branch | tip |
|---|---|
| `CAN_SERVO` | `da63cd18cf200ef2341fb0124fc57cd3ae7911c0` |
| `dev/georgesleen/imu_communication_module` | `f0629372f7e1cb0040fb23b06c51321371057ee6` |
| `emma-refactoring-branch` | `d121eb0000ba0ead2e9c8b376388fb2db37680c3` |
| `rudder-control-model` | `6fe59501acd72d3324acc6bde245348f62ba9ea3` |
| `integration/modules-to-main` | `a361c44db4a41f45f9f50cc5298530e0b8d121df` |

## Had unique commits

Superseded, but the commits below exist nowhere else.

| branch | tip | unique commits |
|---|---|---|
| `adding-tunable-params` | `830693e0946e122a7b4c92c1e48215443e39181a` | 3 |
| `CAN_SERVO_patch_1` | `933aacc0288e16fab9c736dfac0e30d52d340d58` | 1 |
| `controller-merge` | `ef5dec2237c94b7e6d06ac03f961aceef1441b6e` | 2 |
| `data-logging-board` | `c5b00464e987b764eeb7def2ed383d3025467d19` | 6 |
| `dev/gsleen/add_imu_project` | `b4fa649b6800170249b68a492bbb05ff8de63c75` | 1 |
| `imu-controller-merge` | `d9ab436170206fb52738bcf8920aa0e7659cf9a6` | 5 |
| `on-water-test-july-10` | `b84497a45a1ec6ebd31c2247859bb98054dc1e5b` | 1 |
| `sense-working-branch` | `a3945e346fba4d71f95d3347b644df5fdec4e918` | 6 |
| `Wingsail-CAN-Servo` | `ed3cfe34a3d53111afa4bc77b053b9a1d9c6b701` | 1 |
| `wingsail-encoder-patch` | `85ef04fc0cc41de63c672f83f340eb0d75c389a7` | 2 |
| `wingsail-solenoid-test` | `554992fd095be59b17810db0c5fcbc84d5bb096d` | 1 |

`sense-working-branch` is the October 2025 prototype, a sibling of
`sense-integrated` rather than an ancestor. The sense line that continued is
`sense-integrated`.

## Kept

- `can-library`, 17 unique commits, never reviewed
- `pdb-nvic-reset`, one unmerged fix: "Added reset line in error handler"

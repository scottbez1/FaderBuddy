# KiBot resources

KiBot's `resources_dir` global option (set in the `.kibot.yml` files) points here.
At startup KiBot globs `<resources_dir>/fonts/*.ttf` and installs what it finds
into `~/.fonts` *inside its Docker container* — that's the only way KiCad running
under the KiBot Actions can see a custom font, since it can't read the fonts that
`ci/electronics/dependencies.sh` installs on the runner itself.

The glob is flat and non-recursive, so the font has to sit directly in `fonts/`.
`fonts/Righteous-Regular.ttf` is therefore a symlink to the real file in
`ci/fonts/Righteous/`, which is where the runner-side tooling reads it from.

Symptom if this breaks: silkscreen text that uses the Righteous outline font
renders in KiCad's default stroke font in the 3D renders, while the gerbers (which
are plotted on the runner, not in the container) still look correct.

# Bootstrap Introduction

Bootstrap is a css framework that makes css styling a whole lot easier for you. In this codebase, bootstrap's spacing utility is frequently used. Click [here](https://getbootstrap.com/docs/5.3/getting-started/introduction/) to access Bootstrap docs. 


## Bootstrap Spacing Utility
When working with the codebase, you will encounter many css classes such as `px-2` and `m-3`. These classes are a part of the bootstrap framework, and provide a convenient method to space out components. Click [here](https://getbootstrap.com/docs/5.3/utilities/spacing/#margin-and-padding) to read specifics, below is a short summary. 

### Property - first letter
- `m` – margin
- `p` – padding

### Sides Selection - second letter
- `t` – top
- `b` – bottom
- `s` – start
- `e` – end
- `x` – left and right (horizontal)
- `y` – top and bottom (vertical)
- (omitted) – all sides

### Size (spacing)
- `0` – 0 spacing
- `1` – 0.25rem
- `2` – 0.5rem
- `3` – 1rem
- `4` – 1.5rem
- `5` – 3rem
- `auto` – auto margin (only for `m`)

### What the hell is a `rem`:
`rem`, or **r**oot **e**le**m**ent, is a measurement that corresponds to the font size of the root element. By default in most browsers:

```css
html {
    font-size: 16px;
}
```

Therefore, in this case, `1rem = 16px`, `0.5rem = 8px`, etc. 

### Some Examples

| Class       | Description                          |
|-------------|--------------------------------------|
| `m-3`       | Margin on all sides = 1rem           |
| `pt-2`      | Padding top = 0.5rem                 |
| `py-0`      | Padding top & bottom = 0             |
| `ms-auto`   | Margin start = auto                  |
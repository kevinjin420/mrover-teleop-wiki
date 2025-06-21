# Vue Fundamentals

Vue is a versatile and beginner-friendly JavaScript framework. Below is a short guide on writing Vue for this codebase. 

## File Structure

A typical `.vue` file includes;
- A `<template>` block for HTML markup
- A `<script>` block for js logic
- A `<style>` block for css styling

Like this:
```vue
<template>
    <div class="wrapper">
        <!-- Child content here -->
    </div>
</template>
<script>
    // Logic
</script>
<style scoped>
    /* Styles */
</style>
```
> [!NOTE]  
> Vue components can only have one root element inside the `<template>`. If you have more, throw it in a wrapper.

> [!NOTE]   
> You should almost always use `<style scoped>` to limit the css to the current component

## Vue-specific syntax
> [!WARNING]    
> The below content only applies to Vue, and will not work on other JavaScript frameworks or vanilla JavaScript

## Looping Through Data: `v-for`

### Loop over a number

```vue
<div v-for="i in x">
  <button v-on:click="addOnetoX()">Add one to x #{{ i }}</button>
</div>
```

### Loop over an array

```vue
<div v-for="elt in arr">
  {{ elt }}
</div>
```

## Conditional Rendering: `v-if`

**Only render the element if the condition is true.**

```vue
<div v-if="x > 2">
  <p>x is greater than two</p>
</div>
```

## Event Binding: `v-on` / `@`

**Bind a function to an event like `click`.**

```vue
<button v-on:click="addOnetoX()">Click Me</button>
<!-- or shorthand -->
<button @click="addOnetoX()">Click Me</button>
```

## Data Properties

**Define component-local variables using the `data()` function.**

```js
data() {
  return {
    x: 1,
    arr: [1, 2, 3]
  }
}
```

## Methods

**Define functions you want to call from your template or internally.**

```js
methods: {
  addOnetoX() {
    this.x = this.x + 1
  }
}
```

## Computed Properties

**Used for derived or reactive values based on existing data.**

```js
computed: {
  xPlusTwo() {
    return this.x + 2
  }
}
```

## Watchers

**Run code in response to changes in a specific data property.**

```js
watch: {
  x(val) {
    console.log("x has changed")
  }
}
```

## Lifecycle Hooks

### `beforeCreate()`

Runs before the component is initialized.

```js
beforeCreate() {
  // setup logic here
}
```

### `created()`

Runs after the component is initialized, but before DOM is mounted.

```js
created() {
  // fetch data, set up reactive properties, etc.
}
```

### `mounted()`

Runs after the component is added to the DOM.

```js
mounted() {
  // DOM-dependent logic
}
```

### `updated()`

Runs after any DOM update caused by reactive changes.

```js
updated() {
  // respond to reactive updates
}
```

### `unmounted()`

Runs when the component is removed from the DOM. Great for cleanup.

```js
unmounted() {
  // cleanup logic, remove event listeners
}
```

---

## Importing Other Components or Utilities

**Import other files for reuse.**

```js
import { mapGetters } from 'vuex'
import Component from './Component.vue'
```

---

## Child Components

**Register child components that you want to use inside this component.**

```js
components: {
  Component
}
```

---

## Scoped CSS

**Style only this component by using the `scoped` attribute.**

```vue
<style scoped>
.wrapper {
  margin: 1px;
}
.box {
  border: 1px solid grey;
}
</style>
```

---

## Final Component Example

Here’s the full structure for reference:

```vue
<template>
  <div class="wrapper box">
    <div v-for="i in x">
      <button @click="addOnetoX()">Add one to x #{{ i }}</button>
    </div>

    <div v-for="elt in arr">
      {{ elt }}
    </div>

    <div v-if="x > 2">
      <p>x is greater than two</p>
    </div>
  </div>
</template>

<script>
import { mapGetters } from 'vuex'
import Component from './Component.vue'

export default {
  name: 'AutonTask',
  data() {
    return {
      x: 1,
      arr: [1, 2, 3]
    }
  },
  methods: {
    addOnetoX() {
      this.x += 1
    }
  },
  computed: {
    xPlusTwo() {
      return this.x + 2
    }
  },
  watch: {
    x(val) {
      console.log("x has changed")
    }
  },
  beforeCreate() {},
  created() {},
  mounted() {},
  updated() {},
  unmounted() {},
  components: {
    Component
  }
}
</script>

<style scoped>
.wrapper {
  margin: 1px;
}
.box {
  border: 1px solid grey;
}
</style>
```

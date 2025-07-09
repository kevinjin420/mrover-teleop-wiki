# shit to get done this school year
- look at mutiple websockets to increase bandwidth
- point cloud for threejs
- UI styling cleanup (some components look like shit)
- migration to (maybe) tailwindcss?
- fix current buttons: checkbox inconsistent, toggle way too small

# bootstrap -> tailwind

# multiple websockets
- drive
- arm
- auton
- nav
- science
- waypoints


---

## all sockets calls needs to be changed from

```js
this.sendMessage(
    'sa',
    {
        type: 'general',
        mode: this.mode
    }
)
```

to 

```js
this.$store.dispatch('websocket/sendMessage', {
	id: 'general',
	message: {
		type: 'test',
		timestamp: new Date().toISOString(),
	}
})
```
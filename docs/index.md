---
title: Quad4D rebooted
layout: default
---

<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>

$$
\alpha = \Omega
$$

# Documentation

[trajectoires](trajectories)


### 0: Smooth back and forth

This is a composite trajectory made using minsnap polynomials

<img src="graphics/000_back_and_forth.webp" width="800">

<details>
  <summary>Details</summary>
  
  <a href="https://github.com/poine/quad4d_rebooted/blob/main/outputs/000_back_and_forth.csv">csv</a>

  <img src="graphics/000_back_and_forth_flat_out.png" width="800" height="240" title="flat output">
  
  <img src="graphics/000_back_and_forth_state.png" width="800" height="240" title="state">
</details> 



### 1: Circle with intro

A circle at constant speed. The intro and outro are made using minsnap polynomials.
  
<img src="graphics/001_circle_with_intro_slow.webp" width="800">

<details>
  <summary>Details</summary>

* [csv](https://github.com/poine/quad4d_rebooted/blob/main/outputs/001_circle_with_intro_slow.csv)
  
  <img src="graphics/001_circle_with_intro_slow_flat_out.png" width="800" height="240" title="flat output">
  <img src="graphics/001_circle_with_intro_slow_state.png" width="800" height="240" title="state">
</details> 


### 3: Sphere

Analytical trajectory, rotating circle

<img src="graphics/003_sphere.webp" width="800">

<details>
  <summary>Details</summary>
  <img src="graphics/003_sphere_flat_out.png" width="800" height="240" title="flat output">
  <img src="graphics/003_sphere_state.png" width="800" height="240" title="state">
</details> 



### 4: Spatialy indexed circle

Stop-stop trajectory on a geometric circle. Dynamic is minsnap polynomials

<img src="graphics/004_space_indexed_circle1.webp" width="800">
  
<details>
  <summary>Details</summary>
  <img src="graphics/004_space_indexed_circle1_flat_out.png" width="800" height="240" title="flat output">
  <img src="graphics/004_space_indexed_circle1_state.png" width="800" height="240" title="state">
</details> 
  

### 5: Spatialy indexed slalon

Stop-stop slalon trajectory. Geometry is described using splines interpolating waypoints, dynamic using minsnap polynomials.

<img src="graphics/005_space_indexed_waypoints1.webp" width="800">

<details>
  <summary>Details</summary>

  <img src="graphics/005_space_indexed_waypoints1_flat_out.png" width="800" height="240" title="flat output">
  <img src="graphics/005_space_indexed_waypoints1_state.png" width="800" height="240" title="state">
  
</details> 
  
  
### 6: Multivehicle?
  <img src="graphics/006_multivehicle.webp" width="800">
  
<!-- ffmpeg -i /home/poine/2025-10-14\ 00-06-47.mkv -loop 0 /tmp/foo.webp -->

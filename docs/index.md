---
title: Quad4D rebooted
layout: default
---

<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>

$$
\alpha = \Omega
$$

# Documentation


* 0: smooth back and forth

	This is a composite trajectory made using minsnap polynomials

  * [cvs](https://github.com/poine/quad4d_rebooted/blob/main/outputs/000_back_and_forth.csv)

  <img src="https://github.com/poine/quad4d_rebooted/blob/main/outputs/000_back_and_forth_flat_out.png?raw=true" width="800" height="240" title="flat output">
  <img src="https://github.com/poine/quad4d_rebooted/blob/main/outputs/000_back_and_forth_state.png?raw=true" width="800" height="240" title="state">

  <img src="graphics/000_back_and_forth.webp" width="800">



* 1: circle with intro

  A circle at constant speed. The intro and outro are made using minsnap polynomials.
  
  * [cvs](https://github.com/poine/quad4d_rebooted/blob/main/outputs/001_circle_with_intro_slow.csv)
  
  <img src="https://github.com/poine/quad4d_rebooted/blob/main/outputs/001_circle_with_intro_slow_flat_out.png?raw=true" width="800" height="240" title="flat output">
  <img src="https://github.com/poine/quad4d_rebooted/blob/main/outputs/001_circle_with_intro_slow_state.png?raw=true" width="800" height="240" title="state">

  <img src="graphics/001_circle_with_intro_slow.webp" width="800">





ffmpeg -i /home/poine/2025-10-14\ 00-06-47.mkv -loop 0 /tmp/foo.webp

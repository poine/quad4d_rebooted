



test_web:
	cd docs; jekyll serve


src/test_read_bin_traj: src/test_read_bin_traj.c
	gcc -o src/test_read_bin_traj src/test_read_bin_traj.c

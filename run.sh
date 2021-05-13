export ACADOS=/home/jgillis/programs/acados

python python_source/OCP_line.py| tee log_rockit.txt
g++ -g src/main.cpp -g src/generate_path.cpp -g src/generate_bubbles.cpp -g src/call_acados_model.cpp -l acados -l rockitmmap -l  acados_ocp_solver_rockit_model -L ${ACADOS}/lib/ -L . -L c_generated_code/ -I . -I include/ -I ${ACADOS}/include/ -I ${ACADOS}/external/blasfeo/include/ -I ${ACADOS}/external/hpipm/include/ -Wl,-rpath=$(pwd)/c_generated_code -L. -Wl,-rpath=. -o main
./main | tee log_c.txt

meld log_rockit.txt log_c.txt

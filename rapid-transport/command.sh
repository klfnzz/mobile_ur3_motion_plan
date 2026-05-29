export PATH="/root/openrave/bin:$PATH"
export RAPID_TRANSPORT_HEADLESS="${RAPID_TRANSPORT_HEADLESS:-0}"
export MPLCONFIGDIR=/tmp/matplotlib
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export NO_AT_BRIDGE=1
transport.paper pick-demo-ranger-ur3 --scene scenarios/exp3.scenario_ranger.yaml 

python3 scripts/check_contact_pass_rate.py \
  --scene scenarios/exp3.scenario_ranger.yaml \
  --export-summary compare_outputs/exp3_ranger_transport5_transport_compare_summary.json \
  --report-json compare_outputs/contact_check_from_export.json




transport.paper simplify-contact -c analytical_rigidbottle_1 -o bottle_1 -a denso_suction_cup2 \
-T "[[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0.05], [0, 0, 0, 1]]" -r rangerandur3 -v


python /workspaces/Openrave/rapid-transport/scripts/check_retimed_path_geometry.py \
  --export-stem /workspaces/Openrave/rapid-transport/compare_outputs/exp3_ranger_transport 



python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml

python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
--config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
--ik generate

python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/exp_verify.yaml \
  --view  




python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/arm_motion_demo.yaml \
  --ik load \
  --view \
  --view-delay 0.08 \
  --view-substeps 20


  python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/paper_ntu_table_single.yaml \
  --view \
  --view-delay 0.08 \
  --view-substeps 8
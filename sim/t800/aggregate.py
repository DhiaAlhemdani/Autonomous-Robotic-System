import json,glob,numpy as np
rs=[json.load(open(f)) for f in sorted(glob.glob("runs/t800_planar_seed*.json"))]
print("episodes",len(rs))
print("goal",sum(r["goal_reached"] for r in rs),"fell",sum(r["fell"] for r in rs),"viol eps",sum(r["violation_steps"]>0 for r in rs))
print("maxtilt max",round(max(r["max_tilt_deg"] for r in rs),2),"minpelvisZ",round(min(r["min_pelvis_z"] for r in rs),3))
print("minP p5/p50",round(np.percentile([r["min_dist_person"] for r in rs],5),2),round(np.percentile([r["min_dist_person"] for r in rs],50),2))
print("steps p50",np.median([r["steps"] for r in rs]),"time-to-goal s p50",np.median([r["steps"]*0.1 for r in rs if r["goal_reached"]]) if any(r["goal_reached"] for r in rs) else None)
print("solve p50/p99 ms (median over eps)",round(np.median([r["summary"]["solve_p50_ms"] for r in rs])),round(np.median([r["summary"]["solve_p99_ms"] for r in rs])),"max p99",round(max(r["summary"]["solve_p99_ms"] for r in rs)))
print("infeasible fraction mean",round(np.mean([r["summary"]["infeasible_fraction"] for r in rs]),4))
print("track err p50/p99 mm (median over eps)",round(1000*np.median([r["summary"]["track_err_p50_m"] for r in rs]),1),round(1000*np.median([r["summary"]["track_err_p99_m"] for r in rs]),1),"max p99 mm",round(1000*max(r["summary"]["track_err_p99_m"] for r in rs),1))
for r in rs: print(r["seed"],r["goal_reached"],r["steps"],"viol",r["violation_steps"],"minP",round(r["min_dist_person"],2),"tilt",round(r["max_tilt_deg"],2),"inf",round(r["summary"]["infeasible_fraction"],3))

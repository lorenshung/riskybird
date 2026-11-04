/*
 * ref_complementary.cpp -- host float reference for the FcRoCC EST program.
 *
 * Replays a sensor CSV (sim/sensors.csv format) through the flight controller's own
 * ComplementaryEstimator (workloads/flight_controller/src/estimator_complementary.cpp), the C
 * code the EST microcode mirrors, and writes the estimator columns of a replay CSV (u0..u3 are 0:
 * the microcode controller is not modelled here). Configure it as the microcode is built:
 *
 *   c++ -O2 -I../../../../workloads/flight_controller/src -DIMU_OFFSET_Y=-0.016f \
 *       -DFC_FLOW_BODYZ=0|1 ref_complementary.cpp -o ref_complementary -lm
 *   ./ref_complementary SENSORS.csv OUT.csv
 *
 * Two deliberate departures from a firmware build, both to match the microcode:
 *  - the attitude is not seeded from the first accelerometer sample (the RoCC boots at identity
 *    from FCR_INIT); need_seed is cleared after init(), hence the `private` override below;
 *  - IMU_OFFSET_Y must be passed as above (the microcode carries the PCB lever arm; the C
 *    default is 0).
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define private public   /* reach ComplementaryEstimator::need_seed; this TU only */
#include "estimator_complementary.cpp"
#undef private

int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "usage: %s SENSORS.csv OUT.csv\n", argv[0]);
		return 2;
	}
	FILE *fi = fopen(argv[1], "r"), *fo = fopen(argv[2], "w");
	if (!fi || !fo) {
		perror("ref_complementary");
		return 1;
	}
	ComplementaryEstimator est;
	est.init(0.0f, 0.0f, 0.0f);
	est.need_seed = false;
	est.set_airborne(false);

	char line[1024];
	if (!fgets(line, sizeof line, fi)) return 1;   /* header */
	fprintf(fo, "# impl=ComplementaryEstimator float (FC_FLOW_BODYZ=%d, IMU_OFFSET_Y=%g)\n",
		(int)FC_FLOW_BODYZ, (double)IMU_OFFSET_Y);
	fprintf(fo, "step,ex,ey,ez,er1,er2,er3,evx,evy,evz,u0,u1,u2,u3\n");
	int step = 0;
	while (fgets(line, sizeof line, fi)) {
		float d[13];
		char *p = line;
		int n = 0;
		for (; n < 13; n++) {
			char *e;
			d[n] = strtof(p, &e);
			if (e == p) break;
			p = (*e == ',') ? e + 1 : e;
		}
		if (n != 13) continue;
		/* columns: dt,ax,ay,az,gx,gy,gz,fx,fy,fvalid,height,tofvalid,zsp */
		const float acc[3] = {d[1], d[2], d[3]}, gyr[3] = {d[4], d[5], d[6]}, flow[2] = {d[7], d[8]};
		est.update(acc, gyr, flow, d[9] > 0.5f, d[10], d[11] > 0.5f, 0.0f, false, d[0]);
		float s[EST_NSTATES];
		est.get_state(s);
		fprintf(fo, "%d", step);
		for (int i = 0; i < 9; i++) fprintf(fo, ",%.7f", (double)s[i]);
		fprintf(fo, ",0,0,0,0\n");
		step++;
	}
	fclose(fi);
	fclose(fo);
	fprintf(stderr, "ref_complementary: %d steps -> %s\n", step, argv[2]);
	return 0;
}

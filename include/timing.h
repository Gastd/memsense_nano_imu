#include <time.h>
#include <stdio.h>
#include <sys/time.h>

// Subtract timeval
int timeval_subtract (struct timeval* result, struct timeval* x, struct timeval* y)
{
	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_usec < y->tv_usec)
	{
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000)
	{
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

// Open file for timer functions
void timing_init(FILE** timer_fpp, char timer_fn[])
{
	if((*timer_fpp = fopen(timer_fn, "w+")) == NULL)
	{
		printf("ERROR: Couldn't open file %s", timer_fn);
	}
}

// Gets reference time in beginning of function and saves it in t_start
void timing_start(struct timeval* t_start)
{
	gettimeofday(t_start, NULL);
}

// Takes current time and subtracts t_start from it, and saves both t_end and t_diff in text file
void timing_stop(FILE* timer_fp, long int k, struct timeval* t_diff, struct timeval* t_start, struct timeval* t_end)
{
	gettimeofday(t_end, NULL);
	timeval_subtract(t_diff, t_end, t_start);
	
	//printf("%ld.%06ld\t%ld.%06ld\t%ld.%06ld\n", t_diff->tv_sec, t_diff->tv_usec, t_start->tv_sec, t_start->tv_usec, t_end->tv_sec, t_end->tv_usec);
	
	fprintf(timer_fp, "[%ld]\t%lu.%06lu\t%lu.%06lu\t%lu.%06lu\n", k, t_diff->tv_sec, t_diff->tv_usec, t_start->tv_sec, t_start->tv_usec, t_end->tv_sec, t_end->tv_usec);
}

// Close timing function
void timing_close(FILE* timer_fp)
{
	fclose(timer_fp);
}

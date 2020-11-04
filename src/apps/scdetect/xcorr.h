#ifndef SCDETECT_APPS_SCDETECT_XCORR_H_
#define SCDETECT_APPS_SCDETECT_XCORR_H_

#include <seiscomp/core/genericrecord.h>

#include "template.h"

namespace Seiscomp {
namespace detect {

/* Calculate the maximum correlation coefficient and corresponding lag from two
 * demeaned serieses tr1 and tr2. */
bool xcorr(const double *tr1, const int size_tr1, const double *tr2,
           const int size_tr2, const double sampling_freq,
           const double max_lag_samples, Template::MatchResultPtr result);

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_XCORR_H_

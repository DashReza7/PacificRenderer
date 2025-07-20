#pragma once

#ifdef DOUBLE_FLOAT
	using Float = double;
#else
	using Float = float;
#endif // DOUBLE_FLOAT

#ifdef DOUBLE_INT
	using Int = int;
#else
	using Int = long long int;
#endif // DOUBLE_INT



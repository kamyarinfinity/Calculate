#ifndef __CALCULATE_C_INTERFACE_H__
#define __CALCULATE_C_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

struct HCalculate;
typedef struct HCalculate* Expression;

struct calculate_c_library_template {
    Expression (*createExpression)(const char*, const char*, char*);
    Expression (*newExpression)(const char*, const char*);
    void (*freeExpression)(Expression);

    int (*compare)(Expression, Expression);
    const char* (*getExpression)(Expression);
    int (*getVariables)(Expression);

    double (*evaluateArray)(Expression, double*, int, char*);
    double (*evalArray)(Expression, double*, int);
    double (*eval)(Expression, ...);
};

#ifdef __cplusplus
}
#else
extern const struct calculate_c_library_template Calculate;
#endif

#endif

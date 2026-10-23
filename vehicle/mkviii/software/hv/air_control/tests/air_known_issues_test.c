/* known problems we haven't fixed yet. Each check says what the firmware
 * SHOULD do.
 *
 * While a problem is still there, this test prints it and passes so CI stays
 * green. Once it's fixed, this test fails on purpose: move that check into
 * air_paths_test.c as a normal test and delete it here.
 *
 * Nothing is open right now. To add a problem, write a check that returns
 * true once it's fixed (the helpers in air_harness.h make this easy) and add
 * it to issues[] above the end marker */
#include "air_harness.h"

/* One known problem: what it is, and a check that returns true once fixed */
typedef struct {
    const char *summary;
    bool (*fixed)(void);
} issue_t;

/* The list of open problems (empty for now) */
static const issue_t issues[] = {
    {NULL, NULL}, /* end marker; C11 has no empty initializer lists */
};

/* Run each check: still broken prints KNOWN ISSUE, fixed prints FIXED and fails */
int main(void) {
    int open = 0, fixed = 0;
    for (const issue_t *i = issues; i->fixed; ++i) {
        if (i->fixed()) {
            printf("FIXED: %s. Move this check into air_paths_test.c.\n", i->summary);
            ++fixed;
        } else {
            printf("KNOWN ISSUE: %s\n", i->summary);
            ++open;
        }
    }
    if (!open && !fixed) puts("no known issues");
    return fixed ? 1 : 0;
}

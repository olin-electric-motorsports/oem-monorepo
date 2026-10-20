/* Known issues. Each check describes what the firmware SHOULD do.
 *
 * While an issue is still present this test reports it and passes, so CI
 * stays green. Once a fix makes a check pass, this test fails on purpose:
 * move that check into air_paths_test.c as a normal test and delete it here.
 *
 * No issues are open. To track one, write a check that returns true once the
 * firmware behaves correctly, using the helpers in air_harness.h, and list it
 * in issues[] above the end marker. */
#include "vehicle/mkviii/software/air_control/tests/air_harness.h"

typedef struct {
    const char *summary;
    bool (*fixed)(void);
} issue_t;

static const issue_t issues[] = {
    {NULL, NULL}, /* end marker; C11 has no empty initializer lists */
};

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

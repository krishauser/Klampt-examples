
#include <KrisLibrary/robotics/Stability.h>
#include <KrisLibrary/robotics/Contact.h>
#include <vector>
using namespace KrisLibrary;
using namespace std;

int main() {
    vector<ContactPoint> fc_contacts(5);
    fc_contacts[0].x.set(-1,-1,0);
    fc_contacts[0].n.set(0,0,1);
    fc_contacts[0].kFriction = 0.5;
    fc_contacts[1].x.set(1,-1,0);
    fc_contacts[1].n.set(0,0,1);
    fc_contacts[1].kFriction = 0.5;
    fc_contacts[2].x.set(1,1,0);
    fc_contacts[2].n.set(0,0,1);
    fc_contacts[2].kFriction = 0.5;
    fc_contacts[3].x.set(-1,1,0);
    fc_contacts[3].n.set(0,0,1);
    fc_contacts[3].kFriction = 0.5;
    fc_contacts[4].x.set(0,0,1);
    fc_contacts[4].n.set(0,0,-1);
    fc_contacts[4].kFriction = 0.5;

    vector<ContactPoint> stable_contacts(4);
    for(size_t i=0;i<4;i++)
        stable_contacts[i] = fc_contacts[i];
    
    vector<ContactPoint> unstable_contacts(2);
    unstable_contacts[0].x.set(-1,-1,0);
    unstable_contacts[0].n.set(0,1,0);
    unstable_contacts[0].kFriction = 0.5;
    unstable_contacts[1].x.set(1,1,0);
    unstable_contacts[1].n.set(1,0,0);
    unstable_contacts[1].kFriction = 0.5;

    bool res = TestForceClosure(fc_contacts,4);
    printf("FC force closure detected: %d\n",(int)res);
    assert(res);
    res = TestForceClosure(stable_contacts,4);
    printf("Stable force closure detected: %d\n",(int)res);
    assert(!res);
    res = TestForceClosure(unstable_contacts,4);
    printf("Unstable force closure detected: %d\n",(int)res);
    assert(!res);

    res = TestForceClosure(fc_contacts,8);
    printf("FC force closure detected: %d\n",(int)res);
    assert(res);
    res = TestForceClosure(stable_contacts,8);
    printf("Stable force closure detected: %d\n",(int)res);
    assert(!res);
    res = TestForceClosure(unstable_contacts,8);
    printf("Unstable force closure detected: %d\n",(int)res);
    assert(!res);
    return 0;
}
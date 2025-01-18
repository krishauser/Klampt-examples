
#include <Klampt/Modeling/World.h>
#include <KrisLibrary/utils/fileutils.h>

int main() {
    Klampt::WorldModel world;
    if(!world.LoadXML("../../data/baxter_apc.xml")) {
        printf("Error loading world file\n");
        return 1;
    }
    //top level save
    printf("Saving to worldio_test_save.xml\n");
    world.SaveXML("worldio_test_save.xml");
    Klampt::WorldModel world2;
    if(!world2.LoadXML("worldio_test_save.xml")) {
        printf("Error loading saved world file\n");
        return 1;
    }
    //test nested save
    if(!FileUtils::IsDirectory("worldio_test_save2")) {
        if(!FileUtils::MakeDirectory("worldio_test_save2")) {
            printf("Error making directory worldio_test_save2\n");
            return 1;
        }
    }
    printf("Saving to worldio_test_save2/worldio_test_save.xml\n");
    world.SaveXML("worldio_test_save2/worldio_test_save.xml");
    if(!world2.LoadXML("worldio_test_save2/worldio_test_save.xml")) {
        printf("Error loading saved world file\n");
        return 1;
    }
    return 0;
}
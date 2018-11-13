//
// Created by wangl on 11/8/18.
//

#include "main.h"
#include <iostream>

#include "networktables/NetworkTable.h"
#include "tables/ITableListener.h"

class UDPPlannerServer:public ITableListener {


public:
    void ValueChanged(ITable *source, llvm::StringRef key,
                      std::shared_ptr<nt::Value> value, bool isNew) {
        std::cout << "Value Changed" << std::endl;
    }
};

int main() {
    NetworkTable::SetServerMode();
    NetworkTable::SetTeam(1540);


    auto table = NetworkTable::GetTable("myTable");
    UDPPlannerServer dir;
    table->AddTableListener(&dir);
    table->PutNumber("test", 123.456);
}




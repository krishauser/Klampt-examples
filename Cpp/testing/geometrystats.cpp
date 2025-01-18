#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math/random.h>
#include <Klampt/Interface/WorldViewProgram.h>
#include <fstream>
#include <iomanip>
#include <functional>
using namespace Geometry;
using namespace GLDraw;

static const bool CALCULATE_RUNTIMES = true;
static const bool CAPABILITY_GROUPS = true;
static const bool CSV_OUTPUT = true;
static const bool FORMATTED_CSV_OUTPUT = true;
static const bool MARKDOWN_OUTPUT = true;

class TestResults
{
public:
    TestResults() : success(false), timing1(0), timing2(0) {}
    TestResults(const string& op,const string& _obj1,bool _success):timing1(0), timing2(0) {operation=op; object1=_obj1; success = _success; }
    TestResults(const string& op,const string& _obj1,const string& _obj2,bool _success):timing1(0), timing2(0) {operation=op; object1=_obj1; object2=_obj2; success = _success; }
    TestResults(const string& op,const string& _obj1,bool _success,float _timing):timing2(0) {operation=op; object1=_obj1; success = _success; timing1=_timing;}
    TestResults(const string& op,const string& _obj1,const string& _obj2,bool _success,float _timing):timing2(0) {operation=op; object1=_obj1; object2=_obj2; success = _success; timing1=_timing; }
    TestResults(const TestResults& template_) { *this = template_; }
    TestResults(const TestResults& template_,const string& _obj1,bool _success) { *this = template_; object1=_obj1; success = _success; }
    TestResults(const TestResults& template_,const string& _obj1,const string& _obj2,bool _success) { *this = template_; object1=_obj1; object2=_obj2; success = _success; }
    TestResults(const TestResults& template_,const string& _obj1,bool _success,float _timing) { *this = template_; object1=_obj1; success = _success; timing1 = _timing; }
    TestResults(const TestResults& template_,const string& _obj1,const string& _obj2,bool _success,float _timing) { *this = template_; object1=_obj1; success = _success; timing1 = _timing; }
    string operation_string() const { if(operation_condition.empty()) return operation; else return operation + " (" + operation_condition + ")"; }
    string index() const { return operation + ";" + operation_condition + ";" + object1 + ";" + object2; }

    string operation;
    string operation_condition;
    string object1;
    string object2;
    bool success;
    string success_notes;
    float timing1;
    string timing1_condition;
    float timing2;
    string timing2_condition;
};

void WriteMarkdownTable(const vector<vector<string> >& table, ostream& out,int minHeaderRowWidth=0)
{
    size_t ncols = 0;
    for(size_t i=0;i<table.size();i++) {
        Assert(table[i].size() >= 1);
        ncols = Max(ncols,table[i].size());
    }
    size_t max_row_label = 0;
    for(size_t i=0;i<table.size();i++)
        max_row_label = Max(max_row_label,table[i][0].length());
    vector<int> rowWidths(ncols,minHeaderRowWidth);
    if(minHeaderRowWidth == 0) {
        for(size_t i=0;i<table.size();i++) {
            for(size_t j=1;j<table[i].size();j++) {
                rowWidths[j] = Max(rowWidths[j],(int)table[i][j].length());
            }
        }
        for(size_t j=0;j<ncols;j++)
            if(rowWidths[j] < 1) rowWidths[j] = 1;
    }
    for(size_t i=0;i<table.size();i++) {
        const auto& row = table[i];
        out<<"| "<<row[0];
        //pad to longest operation length
        for(size_t j=row[0].length();j<max_row_label;j++)
            out<<" ";
        out<<"| ";
        for(size_t j=1;j<row.size();j++)  {
            out<<row[j];
            if(row[j].back() != ' ') out<<" | ";
            else out<<"| "; 
        }
        //pad columns if needed
        for(size_t j=row.size();j<ncols;j++)
            out<<" |";
        out<<endl;
        if(i==0) { //header row break
            out<<"|";
            for(size_t j=0;j<max_row_label+1;j++)
                out<<"-";
            out<<"|";
            for(size_t j=1;j<ncols;j++) {
                for(int k=0;k<rowWidths[j];k++)
                    out<<"-";
                out<<"|";
            }
            out<<endl;
        }
    }
}

void CapabilityOutput(const vector<string>& labels,const vector<TestResults> results,const char* fn_prefix=NULL)
{
    if(fn_prefix == NULL) fn_prefix = "geometry_capabilities";
    if(CAPABILITY_GROUPS) {
        //multiple instances of the same type can be grouped together
        vector<string> groupMap(labels.size());
        vector<string> groups;
        map<string,string> groupByLabel;
        for(size_t i=0;i<labels.size();i++) {
            vector<string> elems = Split(labels[i]," ");
            Assert(elems.size() > 0);
            if(find(groups.begin(),groups.end(),elems[0]) == groups.end()) {
                groups.push_back(elems[0]);
            }
            groupByLabel[labels[i]] = elems[0];
            groupMap[i] = elems[0];
        }
        if(groups.size() < labels.size()) {
            //remap results
            vector<TestResults> groupResults;
            vector<int> groupCounts;
            groupResults.reserve(results.size());
            groupCounts.reserve(results.size());
            map<string,int> groupResultOrders;
            for(size_t i=0;i<results.size();i++) {
                TestResults gresult(results[i]);
                if(!gresult.object1.empty()) {
                    if(std::count(groups.begin(),groups.end(),results[i].object1) == 0) {
                        Assert(groupByLabel.count(results[i].object1) > 0);
                        gresult.object1 = groupByLabel[results[i].object1];
                    }
                }
                if(!gresult.object2.empty()) {
                    if(std::count(groups.begin(),groups.end(),results[i].object2) == 0) {
                        Assert(groupByLabel.count(results[i].object2) > 0);
                        gresult.object2 = groupByLabel[results[i].object2];
                    }
                }
                string key = gresult.index();
                if(groupResultOrders.count(key) == 0) {
                    int idx = (int)groupResults.size();
                    groupResults.resize(idx+1);
                    groupCounts.resize(idx+1);
                    groupResultOrders[key] = idx;
                    groupResults[idx] = gresult;
                    groupCounts[idx] = 1;
                }
                else {
                    int idx = groupResultOrders[key];
                    groupCounts[idx] ++;
                    if(gresult.success != groupResults[idx].success)
                        groupResults[idx].success_notes = "mixed";
                    //average the timing
                    if(gresult.timing1 > 0) groupResults[idx].timing1 += (gresult.timing1 - groupResults[idx].timing1)/groupCounts[idx];
                    if(gresult.timing2 > 0) groupResults[idx].timing2 += (gresult.timing2 - groupResults[idx].timing2)/groupCounts[idx];
                }
            }
            CapabilityOutput(groups,groupResults);
            return;
        }
    }
    if(CSV_OUTPUT) {
        stringstream ss;
        ss<<fn_prefix<<".csv";
        cout<<"DATA TABLES SAVED TO "<<ss.str()<<endl;
        ofstream out(ss.str().c_str());
        out<<"operation,operation condition,object1,object2,supported,support notes,timing 1,timing condition 1,timing 2,timing condition 2"<<endl;
        for(size_t i=0;i<results.size();i++) {
            out<<results[i].operation<<","<<results[i].operation_condition<<","<<results[i].object1<<","<<results[i].object2<<","<<(results[i].success?"y":"n")<<","<<results[i].success_notes<<",";
            if(results[i].timing1 > 0)
                out<<results[i].timing1;
            out<<","<<results[i].timing1_condition<<",";
            if(results[i].timing2 > 0)
                cout<<results[i].timing2;
            out<<","<<results[i].timing2_condition<<endl;
        }
    }

    map<string,int> labelToIndex;
    for(size_t i=0;i<labels.size();i++) {
        labelToIndex[labels[i]] = (int)i;
    }
    map<string,vector<TestResults> > unaryOperations;
    map<string,Array2D<TestResults> > binaryOperations;
    vector<string> unaryOperationOrder;
    vector<string> binaryOperationOrder;
    for(const auto& res : results) {
        Assert(!res.object1.empty());
        string op = res.operation_string();
        if(res.object2.empty()) {
            if(unaryOperations.count(op) == 0) unaryOperationOrder.push_back(op);
            if(unaryOperations[op].empty())
                unaryOperations[op].resize(labels.size());
            Assert(labelToIndex.count(res.object1) > 0);
            unaryOperations[op][labelToIndex[res.object1]] = res;
        }
        else {
            if(binaryOperations.count(op) == 0) binaryOperationOrder.push_back(op);
            if(binaryOperations[op].empty())
                binaryOperations[op].resize(labels.size(),labels.size());
            Assert(labelToIndex.count(res.object1) > 0);
            Assert(labelToIndex.count(res.object2) > 0);
            //this would be object 1 in row, object 2 in column
            //binaryOperations[op](labelToIndex[res.object1],labelToIndex[res.object2]) = res;
            //this would be object 1 in column, object 2 in row
            binaryOperations[op](labelToIndex[res.object2],labelToIndex[res.object1]) = res;
        }
    }
    //CSV output
    if(FORMATTED_CSV_OUTPUT) {
        stringstream ss;
        ss<<fn_prefix<<"_formatted.csv";
        cout<<"FORMATTED DATA TABLES SAVED TO "<<ss.str()<<endl;
        ofstream out(ss.str().c_str());
        //unary ops
        out<<"SUPPORTED"<<endl;
        out<<"operation";
        for(size_t i=0;i<labels.size();i++) {
            out<<","<<labels[i];
        }
        out<<endl;
        for(const auto& i:unaryOperationOrder) {
            out<<i;
            for(size_t j=0;j<labels.size();j++) {
                const auto& rec = unaryOperations[i][j];
                out<<",";
                if(rec.success) out<<"y";
                else out<<"n";
                if(!rec.success_notes.empty()) out<<" ("<<rec.success_notes<<")";
            }
            out<<endl;
        }
        out<<endl;
        for(const auto& i:binaryOperationOrder) {
            out<<i;
            for(size_t j=0;j<labels.size();j++) {
                out<<","<<labels[j];
            }
            out<<endl;
            for(size_t j=0;j<labels.size();j++) {
                out<<labels[j];
                for(size_t k=0;k<labels.size();k++) {
                    const auto& rec = binaryOperations[i](j,k);
                    out<<",";
                    if(rec.success) out<<"y";
                    else out<<"n";
                    if(!rec.success_notes.empty()) out<<" ("<<rec.success_notes<<")";
                }
                out<<endl;
            }
            out<<endl;
        }

        out<<"TIMING"<<endl;
        out<<"operation";
        for(size_t i=0;i<labels.size();i++) {
            out<<","<<labels[i];
        }
        out<<endl;
        for(const auto& i:unaryOperationOrder) {
            out<<i;
            for(size_t j=0;j<labels.size();j++) {
                const auto& rec = unaryOperations[i][j];
                if(rec.timing1 > 0)
                    out<<","<<rec.timing1;
                else
                    out<<",";
            }
            out<<endl;
        }
        out<<endl;
        for(const auto& i:binaryOperationOrder) {
            out<<i;
            for(size_t j=0;j<labels.size();j++) {
                out<<","<<labels[j];
            }
            out<<endl;
            for(size_t j=0;j<labels.size();j++) {
                out<<labels[j];
                for(size_t k=0;k<labels.size();k++) {
                    const auto& rec = binaryOperations[i](j,k);
                    if(rec.timing1 > 0)
                        out<<","<<rec.timing1;
                    else
                        out<<",";
                }
                out<<endl;
            }
            out<<endl;
        }
    }
    if(MARKDOWN_OUTPUT) {
        //Markdown output
        stringstream ss;
        ss<<fn_prefix<<"_markdown.md";
        cout<<"MARKDOWN DATA TABLES SAVED TO "<<ss.str()<<endl;
        ofstream out(ss.str().c_str());
        out<<"**Supported**"<<endl;
        vector<vector<string > > unaryTable;
        unaryTable.resize(1);
        unaryTable[0].resize(labels.size()+1);
        unaryTable[0][0] = "operation";
        for(size_t i=0;i<labels.size();i++) {
            unaryTable[0][i+1] = labels[i];
        }
        vector<string> superscripts = {"⁰","¹","²","³","⁴","⁵","⁶","⁷","⁸","⁹"};
        std::map<string,int> reasons;
        for(const auto& i:unaryOperationOrder) {
            unaryTable.resize(unaryTable.size()+1);
            unaryTable.back().resize(labels.size()+1);
            unaryTable.back()[0] = i;
            for(size_t j=0;j<labels.size();j++) {
                const auto& rec = unaryOperations[i][j];
                unaryTable.back()[j+1] = (rec.success?"✔️":"❌");
                if(!rec.success_notes.empty()) {
                    if(reasons.count(rec.success_notes) == 0)
                        reasons[rec.success_notes] = (int)reasons.size()+1;
                    unaryTable.back()[j+1] += superscripts[reasons[rec.success_notes]];
                }
                else {
                    unaryTable.back()[j+1] += " "; //for spacing
                }
                if(rec.success) unaryTable.back()[j+1] += " "; //for spacing
            }
        }
        WriteMarkdownTable(unaryTable,out,4);
        if(reasons.size() > 0) {
            out<<endl;
            vector<string> reasonsInv(reasons.size());
            for(auto i:reasons) {
                reasonsInv[i.second-1] = i.first;
            }
            for(size_t i=0;i<reasonsInv.size();i++) {
                out<<i+1<<". "<<reasonsInv[i]<<endl;
            }
            out<<endl;
        }

        //binary tables -- reasons given in subscript
        for(const auto& op:binaryOperationOrder) {
            reasons.clear();
            out<<"*"<<op<<"*"<<endl;
            vector<vector<string > > binaryTable(1);
            binaryTable[0].resize(labels.size()+1);
            for(size_t j=0;j<labels.size();j++) 
                binaryTable[0][j+1] = labels[j];
            for(size_t i=0;i<labels.size();i++) {
                binaryTable.resize(binaryTable.size()+1);
                binaryTable.back().resize(labels.size()+1);
                binaryTable.back()[0] = labels[i];
                for(size_t j=0;j<labels.size();j++) {
                    const auto& rec = binaryOperations[op](i,j);
                    binaryTable.back()[j+1] = (rec.success?"✔️":"❌");
                    if(!rec.success_notes.empty()) {
                        if(reasons.count(rec.success_notes) == 0)
                            reasons[rec.success_notes] = (int)reasons.size()+1;
                        binaryTable.back()[j+1] += superscripts[reasons[rec.success_notes]];
                    }
                    else {
                        binaryTable.back()[j+1] += " "; //for spacing
                    }
                    if(rec.success) binaryTable.back()[j+1] += " "; //for spacing
                }
            }
            WriteMarkdownTable(binaryTable,out,4);
            //one footnote list per table  
            if(reasons.size() > 0) {
                out<<endl;
                vector<string> reasonsInv(reasons.size());
                for(auto i:reasons) {
                    reasonsInv[i.second-1] = i.first;
                }
                for(size_t i=0;i<reasonsInv.size();i++) {
                    out<<i+1<<". "<<reasonsInv[i]<<endl;
                }
            }
            out<<endl;
        }
        out<<endl;

        out<<"**Timing** (units in ms)"<<endl;
        int rowIndex = 1;
        for(const auto& i:unaryOperationOrder) {
            unaryTable[rowIndex][0] = i;
            for(size_t j=0;j<labels.size();j++) {
                const auto& rec = unaryOperations[i][j];
                stringstream ss;
                if(rec.timing1 > 0)
                    ss<<std::fixed<<setprecision(3)<<rec.timing1*1000;
                else
                    ss<<"      ";
                unaryTable[rowIndex][j+1] = ss.str();
            }
            rowIndex ++;
        }
        WriteMarkdownTable(unaryTable,out,7);
        for(const auto& op:binaryOperationOrder) {
            out<<endl;
            out<<"**"<<op<<"**"<<endl;
            vector<vector<string > > binaryTable(1);
            binaryTable[0].resize(labels.size()+1);
            for(size_t j=0;j<labels.size();j++) 
                binaryTable[0][j+1] = labels[j];
            for(size_t i=0;i<labels.size();i++) {
                binaryTable.resize(binaryTable.size()+1);
                binaryTable.back().resize(labels.size()+1);
                binaryTable.back()[0] = labels[i];
                for(size_t j=0;j<labels.size();j++) {
                    const auto& rec = binaryOperations[op](i,j);
                    stringstream ss;
                    if(rec.timing1 > 0)
                        ss<<std::fixed<<setprecision(3)<<rec.timing1*1000;
                    else
                        ss<<"      ";
                    binaryTable.back()[j+1] = ss.str();
                }
            }
            WriteMarkdownTable(binaryTable,out,7);
        }
    }

}

float TimeIt(const std::function<void ()>& func, int num_tries, float max_time=0.5, float min_time=0.001) {
    Timer timer;
    int num_total = 0;
    while(timer.ElapsedTime() < min_time) {
        for(int i=0;i<num_tries;i++) {
            func();
            num_total += 1;
            if(num_total < num_tries && num_total == 1 && timer.ElapsedTime() > max_time) { //very long runs
                printf("Early termination of timing loop, ran %d iterations of %d\n",num_total,num_tries);
                break;
            }
            if(num_total < num_tries && num_total % 10 == 0 && timer.ElapsedTime() > max_time) {
                printf("Early termination of timing loop, ran %d iterations of %d\n",num_total,num_tries);
                break;
            }
        }
    }
    return timer.ElapsedTime() / num_total;
}


void CapabilityMatrix(const vector<AnyGeometry3D>& geoms,const vector<string>& labels,const char* fn_prefix=NULL)
{
    Assert(geoms.size() == labels.size());
    vector<TestResults> results;
    TestResults template_;
    vector<vector<shared_ptr<AnyGeometry3D> > > conversionResults;

    vector<AnyCollisionGeometry3D> cgeoms(geoms.size());
    RigidTransform T0;
    T0.R.setRotateX(DtoR(30.0));
    T0.t.set(1.0,0.0,0.0);
    //initialization

    printf("BEGINNING initialize collider TEST\n");
    for(size_t i=0;i<geoms.size();i++) {
        cgeoms[i] = AnyCollisionGeometry3D(geoms[i]);
        float timing = TimeIt([&cgeoms,i]() {
            cgeoms[i].InitCollisionData();
            },10);
        results.push_back(TestResults("initialize collider",labels[i],true,timing));
        cgeoms[i].SetTransform(T0);
    }

    // printf("BEGINNING set transform TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     float timing = TimeIt([&cgeoms,i,&T0]() {
    //         RigidTransform T=T0;
    //         T.t.x += Rand(-1.5,1.5);
    //         cgeoms[i].SetTransform(T);
    //         },100);
    //     cgeoms[i].SetTransform(T0);
    //     results.push_back(TestResults("set transform",labels[i],true,timing));
    // }
    
    // printf("BEGINNING transform TEST\n");
    // template_.operation = "transform";
    // template_.operation_condition = "rigid";
    // for(size_t i=0;i<geoms.size();i++) {
    //     AnyGeometry3D temp = geoms[i];
    //     bool success = temp.Transform(T0);
    //     float timing = 0;
    //     if(success) {
    //         timing = TimeIt([&temp,&T0]() {
    //             RigidTransform T=T0;
    //             T.t.x += Rand(-1.5,1.5);
    //             temp.Transform(T);
    //             },100);
    //     }
    //     results.push_back(TestResults(template_,labels[i],success,timing));
    // }
    // template_.operation = "transform";
    // template_.operation_condition = "nonuniform scale";
    // for(size_t i=0;i<geoms.size();i++) {
    //     AnyGeometry3D temp = geoms[i];
    //     Matrix4 m;
    //     m.setIdentity();
    //     m(0,0) = 2.0;
    //     m(1,1) = 0.75;
    //     bool success = temp.Transform(m);
    //     float timing = 0;
    //     if(success) {
    //         timing = TimeIt([&geoms,i]() {
    //             Matrix4 m;
    //             m.setIdentity();
    //             m(0,0) = Rand(1.0,2.0);
    //             m(1,1) = Rand(0.5,1.0);
    //             AnyGeometry3D temp = geoms[i];
    //             temp.Transform(m);
    //             },100) - TimeIt([&geoms,i]() { AnyGeometry3D temp=geoms[i]; },100);
    //     }
    //     results.push_back(TestResults(template_,labels[i],success,timing));
    // }

    // template_.operation = "transform";
    // template_.operation_condition = "rotation + uniform scale";
    // for(size_t i=0;i<geoms.size();i++) {
    //     AnyGeometry3D temp = geoms[i];
    //     Matrix4 m(T0);
    //     for(int p=0;p<3;p++) 
    //         for(int q=0;q<3;q++) 
    //             m(p,q) *= 1.5;
    //     bool success = temp.Transform(m);
    //     float timing = 0;
    //     if(success) {
    //         timing = TimeIt([&geoms,i,&T0]() {
    //             Matrix4 m(T0);
    //             for(int p=0;p<3;p++) 
    //                 for(int q=0;q<3;q++) 
    //                     m(p,q) *= Rand(1.0,2.0);
    //             AnyGeometry3D temp = geoms[i];
    //             temp.Transform(m);
    //             },100) - TimeIt([&geoms,i]() { AnyGeometry3D temp=geoms[i]; },100);
    //     }
    //     results.push_back(TestResults(template_,labels[i],success,timing));
    // }

    // printf("BEGINNING bb TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     AABB3D bb = cgeoms[i].GetAABB();
    //     AABB3D bbt = cgeoms[i].GetAABBTight();
    //     //this operation is definitely successful... don't worry about checking success
    //     float timing = TimeIt([&cgeoms,i,&T0]() {
    //         RigidTransform T=T0;
    //         T.t.x += Rand(-1.5,1.0); 
    //         cgeoms[i].SetTransform(T);
    //         cgeoms[i].GetAABB();
    //         },100);
    //     cgeoms[i].SetTransform(T0);
    //     if(bb.bmin.x == -Inf) results.push_back(TestResults(template_,labels[i],false,timing));
    //     else if(bb.bmin == bbt.bmin && bb.bmax == bbt.bmax) {
    //         TestResults res("bb",labels[i],true,timing);
    //         res.success_notes = "exact";
    //         results.push_back(res);
    //     }
    //     else results.push_back(TestResults("bb",labels[i],true,timing));
    // }

    // printf("BEGINNING getelement TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     if(geoms[i].NumElements() > 0) {
    //         shared_ptr<Geometry3D> elem = geoms[i].data->GetElement(0);
    //         if(elem) results.push_back(TestResults("getelement",labels[i],true));
    //         else results.push_back(TestResults("getelement",labels[i],false));
    //     }
    //     else
    //         results.push_back(TestResults("getelement",labels[i],false));
    // }

    // printf("BEGINNING remesh TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     AnyGeometry3D temp;
    //     float timing = 0;
    //     bool success = geoms[i].Remesh(0.1,temp);
    //     if(success) {
    //         timing = TimeIt([&geoms,i]() {
    //             AnyGeometry3D temp;
    //             geoms[i].Remesh(0.1,temp);
    //             },10);
    //     }
    //     results.push_back(TestResults("remesh",labels[i],success,timing));
    // }

    // printf("BEGINNING extract ROI TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     AABB3D bb;
    //     bb.bmin.set(-1.5,0.5,0.5);
    //     bb.bmax.set(1.5,1.5,1.5);
    //     AnyCollisionGeometry3D ctemp;
    //     AnyGeometry3D temp;
    //     if(cgeoms[i].ExtractROI(bb,ctemp)) {
    //         float timing = TimeIt([&cgeoms,i]() {
    //             AABB3D bb;
    //             bb.bmin.x = Rand(-2.0,1.0);
    //             bb.bmin.y = Rand(0.5,1.0);
    //             bb.bmin.z = Rand(0.5,1.0);
    //             bb.bmax.x = Rand(1.0,3.0);
    //             bb.bmax.y = Rand(1.0,1.5);
    //             bb.bmax.z = Rand(0.5,1.0);
    //             AnyCollisionGeometry3D ctemp;
    //             cgeoms[i].ExtractROI(bb,ctemp);
    //             },100);
    //         results.push_back(TestResults("extract ROI",labels[i],true,timing));
    //     }
    //     else if(geoms[i].ExtractROI(bb,temp)) {
    //         float timing = TimeIt([&geoms,i]() {
    //             AABB3D bb;
    //             bb.bmin.x = Rand(-2.0,1.0);
    //             bb.bmin.y = Rand(0.5,1.0);
    //             bb.bmin.z = Rand(0.5,1.0);
    //             bb.bmax.x = Rand(1.0,3.0);
    //             bb.bmax.y = Rand(1.0,1.5);
    //             bb.bmax.z = Rand(0.5,1.0);
    //             AnyGeometry3D temp;
    //             geoms[i].ExtractROI(bb,temp);
    //             },100);
    //         TestResults res("extract ROI",labels[i],true,timing);
    //         res.success_notes = "axis-aligned";
    //         results.push_back(res);
    //     }
    //     else
    //         results.push_back(TestResults("extract ROI",labels[i],false));
    // }

    // printf("BEGINNING slice TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     AnyCollisionGeometry3D temp;
    //     RigidTransform T=T0;
    //     T.t.z += 0.4;
    //     float timing = 0;
    //     bool success = cgeoms[i].Slice(T,temp,0.01);
    //     if(success) {
    //         timing = TimeIt([&cgeoms,i,&T0]() {
    //             AnyCollisionGeometry3D temp;
    //             RigidTransform T=T0;
    //             T.t.z += Rand(-0.5,0.5);
    //             cgeoms[i].Slice(T,temp,0.01);
    //             },10);
    //     }
    //     results.push_back(TestResults("slice",labels[i],success,timing));
    // }

    // printf("BEGINNING union TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     vector<AnyGeometry3D> temp(2);
    //     temp[0] = geoms[i];
    //     temp[1] = geoms[i];
    //     temp[1].Transform(T0);
    //     AnyGeometry3D temp2;
    //     temp2.Union(temp);
    //     float timing = 0;
    //     bool success = (temp2.type != AnyGeometry3D::Type::Group);
    //     if(success) {
    //         timing = TimeIt([&geoms,i,&T0]() {
    //             vector<AnyGeometry3D> temp(2);
    //             temp[0] = geoms[i];
    //             temp[1] = geoms[i];
    //             RigidTransform T=T0;
    //             T.t.x += Rand(-0.5,0.5);
    //             temp[1].Transform(T);
    //             AnyGeometry3D temp2;
    //             temp2.Union(temp);
    //             },10);
    //     }
    //     results.push_back(TestResults("union",labels[i],success,timing));
    // }

    // printf("BEGINNING support TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     Vector3 dir(0.3,0.5,0.7), pt;
    //     float timing = 0;
    //     bool success = cgeoms[i].Support(dir,pt);
    //     if(success) {
    //         timing = TimeIt([&cgeoms,i]() {
    //             Vector3 dir(0.3,0.5,0.7), pt;
    //             dir.x = Rand(-0.5,0.5);
    //             cgeoms[i].Support(dir,pt);
    //             },100);
    //     }
    //     results.push_back(TestResults("support",labels[i],success,timing));
    // }

    // printf("BEGINNING ray cast TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     Ray3D r;
    //     r.source.set(-0.5,0.0,0.0);
    //     r.direction.set(0.3,0.5,0.7);
    //     Real dist;
    //     int elem;
    //     Vector3 pt;
    //     float timing = 0;
    //     bool success = cgeoms[i].collider->RayCast(r,0.0,dist,elem);
    //     if(success) {
    //         timing = TimeIt([&cgeoms,i]() {
    //             Ray3D r;
    //             r.source.set(Rand(-1.5,1.5),0.0,0.0);
    //             r.direction.set(Rand(-0.5,0.5),0.5,0.7);
    //             Real dist;
    //             int elem;
    //             cgeoms[i].collider->RayCast(r,0.0,dist,elem);
    //             },100);
    //     }
    //     results.push_back(TestResults("ray cast",labels[i],success,timing));
    // }
    
    // printf("BEGINNING point containment TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     Vector3 pt(0.5,0.5,0.5);
    //     bool res;
    //     float timing = 0;
    //     bool success = cgeoms[i].collider->Contains(pt,res);
    //     if(success) {
    //         timing = TimeIt([&cgeoms,i]() {
    //             Vector3 pt(0.5,0.5,0.5);
    //             bool res;
    //             pt.x = Rand(-1.5,1.5);
    //             cgeoms[i].collider->Contains(pt,res);
    //             },100);
    //     }
    //     results.push_back(TestResults("point containment",labels[i],success,timing));
    // }

    // printf("BEGINNING point distance TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     Vector3 pt(0.5,0.5,0.5);
    //     Real res = cgeoms[i].Distance(pt);
    //     float timing = 0;
    //     bool success = !IsInf(res);
    //     if(success) {
    //         timing = TimeIt([&cgeoms,i]() {
    //             Vector3 pt(0.5,0.5,0.5);
    //             pt.x = Rand(-1.5,1.5);
    //             cgeoms[i].Distance(pt);
    //             },100);
    //     }
    //     results.push_back(TestResults("point distance",labels[i],success,timing));
    // }

    // printf("BEGINNING convert TEST\n");
    // for(int i=0;i<(int)Geometry3D::Type::Group;i++) {
    //     conversionResults.resize(conversionResults.size()+1);
    //     for(size_t j=0;j<geoms.size();j++) {
    //         shared_ptr<AnyGeometry3D> temp = make_shared<AnyGeometry3D>();
    //         bool success;
    //         success = geoms[j].Convert((Geometry3D::Type)i,*temp);
    //         if(!success)
    //             temp.reset();
    //         float timing = 0;
    //         if(success) {
    //             timing = TimeIt([&geoms,j,i]() {
    //                 shared_ptr<AnyGeometry3D> temp = make_shared<AnyGeometry3D>();
    //                 geoms[j].Convert((Geometry3D::Type)i,*temp);
    //                 },10);
    //         }
    //         string target=Geometry3D::TypeName((Geometry3D::Type)i);
    //         results.push_back(TestResults("convert",labels[j],target,success,timing));
    //         conversionResults.back().push_back(temp);
    //     }
    // }

    // printf("BEGINNING collision TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     for(size_t j=i;j<geoms.size();j++) {
    //         printf("Testing collision between %s and %s\n",labels[i].c_str(),labels[j].c_str());
    //         AnyCollisionGeometry3D temp = cgeoms[j];
    //         RigidTransform T = temp.GetTransform();
    //         T.t.x += 0.5;
    //         temp.SetTransform(T);
    //         bool res;
    //         bool success;
    //         //usually would just use cgeoms[j].Collides(temp) but if the test is not implemented Collides will return false.
    //         if(cgeoms[i].collider->Collides(temp.collider.get(),res)) {
    //             printf("%s collides with %s\n",cgeoms[i].TypeName(),temp.TypeName());
    //             success = true;
    //         }
    //         else if(temp.collider->Collides(cgeoms[i].collider.get(),res)) { 
    //             printf("%s collides with %s\n",temp.TypeName(),cgeoms[i].TypeName());
    //             success = true;
    //         }
    //         else {
    //             success = false;
    //         }
    //         float timing = 0;
    //         if(success) {
    //             timing = TimeIt([&cgeoms,&temp,i,&T0]() {
    //                 RigidTransform T = T0;
    //                 T.t.x += Rand(-1.5,1.5);
    //                 T.t.z += 0.4;
    //                 temp.SetTransform(T);
    //                 cgeoms[i].Collides(temp);
    //                 },100);
    //         }
    //         results.push_back(TestResults("collides",labels[i],labels[j],success,timing));
    //         if(i!=j)
    //             results.push_back(TestResults("collides",labels[j],labels[i],success,timing));
    //     }
    // }

    //early output for debugging
    // // CapabilityOutput(labels,results,fn_prefix);
    // // return;

    printf("BEGINNING within distance TEST\n");
    for(size_t i=0;i<geoms.size();i++) {
        for(size_t j=i;j<geoms.size();j++) {
            AnyCollisionGeometry3D temp = cgeoms[j];
            RigidTransform T = temp.GetTransform();
            T.t.x += 0.5;
            temp.SetTransform(T);
            bool res;
            //usually would just use cgeoms[i].Within(temp,0.1) but if the test is not implemented WithinDistance will return false.
            bool success = cgeoms[i].collider->WithinDistance(temp.collider.get(),0.1,res) || temp.collider->WithinDistance(cgeoms[i].collider.get(),0.1,res);
            float timing = 0;
            if(success) {
                timing = TimeIt([&cgeoms,&temp,i,&T0]() {
                    RigidTransform T = T0;
                    T.t.x += Rand(-1.5,1.5);
                    T.t.z + 0.4;
                    temp.SetTransform(T);
                    cgeoms[i].WithinDistance(temp,0.1);
                    },100);
            }
            results.push_back(TestResults("within distance",labels[i],labels[j],success,timing));
            if(i!=j)
                results.push_back(TestResults("within distance",labels[j],labels[i],success,timing));
        }
    }

    printf("BEGINNING distance TEST\n");
    for(size_t i=0;i<geoms.size();i++) {
        for(size_t j=i;j<geoms.size();j++) {
            AnyCollisionGeometry3D temp = cgeoms[j];
            RigidTransform T = temp.GetTransform();
            T.t.x += 0.5;
            temp.SetTransform(T);
            DistanceQuerySettings settings;
            DistanceQueryResult res;
            //usually would just use cgeoms[i].Distance(temp,settinsg,res) but if the test is not implemented Distance will return Inf.
            bool success = cgeoms[i].collider->Distance(temp.collider.get(),settings,res) || temp.collider->Distance(cgeoms[i].collider.get(),settings,res); 
            float timing = 0;
            if(success) {
                timing = TimeIt([&cgeoms,&temp,i,&T0]() {
                    DistanceQuerySettings settings;
                    RigidTransform T = T0;
                    T.t.x += Rand(-1.5,1.5);
                    T.t.z + 0.4;
                    temp.SetTransform(T);
                    cgeoms[i].Distance(temp,settings);
                    },100);
            }
            results.push_back(TestResults("distance",labels[i],labels[j],success,timing));
            if(i!=j)
                results.push_back(TestResults("distance",labels[j],labels[i],success,timing));
        }
    }

    // printf("BEGINNING contacts TEST\n");
    for(size_t i=0;i<geoms.size();i++) {
        for(size_t j=i;j<geoms.size();j++) {
            AnyCollisionGeometry3D temp = cgeoms[j];
            RigidTransform T = temp.GetTransform();
            T.t.x += 0.5;
            temp.SetTransform(T);
            ContactsQuerySettings settings;
            settings.padding1 = 0.05;
            settings.padding2 = 0.02;
            ContactsQueryResult res;
            bool success = (cgeoms[i].collider->Contacts(temp.collider.get(),settings,res) || temp.collider->Contacts(cgeoms[i].collider.get(),settings,res));    
            float timing = 0;
            if(success) {
                vector<RigidTransform> Ts_close;
                for(int k=0;k<1000;k++) {
                    RigidTransform T = T0;
                    T.t.x += Rand(-1.5,1.5);
                    T.t.z + 0.4;
                    temp.SetTransform(T);
                    if(cgeoms[i].WithinDistance(temp,settings.padding1+settings.padding2) && !cgeoms[i].Collides(temp)) {
                      Ts_close.push_back(T);
                      if(Ts_close.size() > 100)
                        break;
                    }
                }
                if(Ts_close.empty()) {
                    printf("WARNING: could not sample any transforms in which the geometries are touching (%s vs %s)\n",labels[i].c_str(),labels[j].c_str());
                }
                else {
                    printf("Found %d transforms in which the geometries are touching\n",Ts_close.size());
                    timing = TimeIt([&cgeoms,&temp,i,&Ts_close]() {
                        ContactsQuerySettings settings;
                        RigidTransform T = Ts_close[RandInt(Ts_close.size())];
                        temp.SetTransform(T);
                        cgeoms[i].Contacts(temp,settings);
                        },100);
                }
            }
            results.push_back(TestResults("contacts",labels[i],labels[j],success,timing));
            if(i!=j)
                results.push_back(TestResults("contacts",labels[j],labels[i],success,timing));
        }
    }

    // printf("BEGINNING merge TEST\n");
    // for(size_t i=0;i<geoms.size();i++) {
    //     for(size_t j=0;j<geoms.size();j++) {
    //         AnyCollisionGeometry3D temp = cgeoms[j];
    //         RigidTransform T = temp.GetTransform();
    //         T.t.x += 0.5;
    //         temp.SetTransform(T);
    //         bool success = temp.Merge(cgeoms[i]);
    //         float timing = 0;
    //         if(success) {
    //             timing = TimeIt([&cgeoms,&temp,i,j,&T0]() {
    //                 RigidTransform T = T0;
    //                 T.t.x += Rand(-1.5,1.5);
    //                 temp = cgeoms[j];
    //                 temp.SetTransform(T);
    //                 temp.Merge(cgeoms[i]);
    //                 },10) - TimeIt([&cgeoms,&temp,j,&T0]() {
    //                     temp = cgeoms[j];
    //                     temp.SetTransform(T0);
    //                 },10);
    //         }
    //         results.push_back(TestResults("merge",labels[j],labels[i],success,timing));
    //     }
    // }

    CapabilityOutput(labels,results,fn_prefix);
}

void GenerateCapabilityMatrix()
{
    vector<AnyGeometry3D> geoms(9);
    vector<string> labels(9);
    Sphere3D s;
    s.radius = 0.4;
    s.center.set(-1.5,0.4,2.0);
    geoms[0] = AnyGeometry3D(GeometricPrimitive3D(s));
    labels[0] = "Primitive (sphere)";
    Box3D b;
    b.dims.set(0.5,0.25,0.75);
    b.origin.set(-1.75,0.375,1.70);
    b.xbasis.set(1,0,0);
    b.ybasis.set(0,1,0);
    b.zbasis.set(0,0,1);
    geoms[1] = AnyGeometry3D(GeometricPrimitive3D(b));
    labels[1] = "Primitive (box)";
    bool res = geoms[0].Convert(Geometry3D::Type::ConvexHull,geoms[2]);
    Assert(res);
    labels[2] = "ConvexHull";
    res = geoms[0].Convert(Geometry3D::Type::TriangleMesh,geoms[3]);
    Assert(res);
    labels[3] = "TriangleMesh";
    res = geoms[3].Convert(Geometry3D::Type::PointCloud,geoms[4]);
    Assert(res);
    labels[4] = "PointCloud";
    res = geoms[3].Convert(Geometry3D::Type::ImplicitSurface,geoms[5]);
    Assert(res);
    labels[5] = "ImplicitSurface";
    res = geoms[0].Convert(Geometry3D::Type::OccupancyGrid,geoms[6]);
    Assert(res);
    labels[6] = "OccupancyGrid";
    res = geoms[0].Convert(Geometry3D::Type::Heightmap,geoms[7]);
    Assert(res);
    labels[7] = "Heightmap (orthographic)";
    geoms[8] = geoms[7];
    auto& hm = geoms[8].AsHeightmap();
    hm.viewport.perspective = true;
    hm.Shift(2.0);
    for(int i=0;i<hm.heights.m;i++)
        for(int j=0;j<hm.heights.n;j++)
            if(hm.heights(i,j) == 2.0)
                hm.heights(i,j) = 0;
    hm.viewport.pose.t.z -= 2.0;
    hm.viewport.fx *= 2.0;
    hm.viewport.fy *= 2.0;
    hm.viewport.ori = Camera::CameraConventions::OpenCV;
    labels[8] = "Heightmap (perspective)";
    //CapabilityMatrix(geoms,labels);

    if(CALCULATE_RUNTIMES) {
        //calculate run-time data on simple geometries
        vector<string> simple_labels(7);
        vector<AnyGeometry3D> simple_geoms(7);
        simple_labels[0] = "Primitive";
        simple_geoms[0] = geoms[0];  //sphere
        simple_labels[1] = "ConvexHull";
        res = geoms[1].Convert(Geometry3D::Type::ConvexHull,simple_geoms[1]);
        Assert(res);
        simple_labels[2] = "TriangleMesh";
        res = geoms[0].Convert(Geometry3D::Type::TriangleMesh,simple_geoms[2],0.03);
        Assert(res);
        simple_labels[3] = "PointCloud";
        res = geoms[3].Convert(Geometry3D::Type::PointCloud,simple_geoms[3],0.06);
        Assert(res);
        simple_labels[4] = "ImplicitSurface";
        res = geoms[0].Convert(Geometry3D::Type::ImplicitSurface,simple_geoms[4],0.8/61);
        simple_labels[5] = "OccupancyGrid";
        res = simple_geoms[4].Convert(Geometry3D::Type::OccupancyGrid,simple_geoms[5]);
        Assert(res);
        simple_labels[6] = "Heightmap";
        res = geoms[7].Remesh(0.8/256,simple_geoms[6]);
        Assert(res);
        
        vector<string> complex_labels(7);
        vector<AnyGeometry3D> complex_geoms(7);
        complex_labels[0] = "Primitive";
        complex_geoms[0] = geoms[1];  //box
        complex_labels[1] = "ConvexHull";
        res = geoms[0].Convert(Geometry3D::Type::ConvexHull,complex_geoms[1],0.01);
        Assert(res);
        complex_labels[2] = "TriangleMesh";
        res = geoms[0].Convert(Geometry3D::Type::TriangleMesh,complex_geoms[2],0.003);
        Assert(res);
        complex_labels[3] = "PointCloud";
        res = geoms[3].Convert(Geometry3D::Type::PointCloud,complex_geoms[3],0.005);
        Assert(res);
        complex_labels[4] = "ImplicitSurface";
        res = geoms[0].Convert(Geometry3D::Type::ImplicitSurface,complex_geoms[4],0.8/253);
        complex_labels[5] = "OccupancyGrid";
        res = complex_geoms[4].Convert(Geometry3D::Type::OccupancyGrid,complex_geoms[5]);
        Assert(res);
        complex_labels[6] = "Heightmap";
        res = geoms[7].Remesh(0.8/2048,complex_geoms[6]);
        Assert(res);

        printf("Saved to simple_geometry_X files\n");
        printf("  Convex hull vertices %d\n",(int)simple_geoms[1].AsConvexHull().GetPoints().size());
        printf("  Triangle mesh triangles %d\n",(int)simple_geoms[2].AsTriangleMesh().tris.size());
        printf("  PointCloud points %d\n",(int)simple_geoms[3].AsPointCloud().points.size());
        const auto& surf=simple_geoms[4].AsImplicitSurface();
        printf("  Implicit surface %d %d %d\n",surf.value.m,surf.value.n,surf.value.p);
        const auto& hm=simple_geoms[6].AsHeightmap();
        printf("  Heightmap %d %d\n",hm.heights.m,hm.heights.n);

        printf("Saved to complex_geometry_X files\n");
        printf("  Convex hull vertices %d\n",(int)complex_geoms[1].AsConvexHull().GetPoints().size());
        printf("  Triangle mesh triangles %d\n",(int)complex_geoms[2].AsTriangleMesh().tris.size());
        printf("  PointCloud points %d\n",(int)complex_geoms[3].AsPointCloud().points.size());
        const auto& surf2=complex_geoms[4].AsImplicitSurface();
        printf("  Implicit surface %d %d %d\n",surf2.value.m,surf2.value.n,surf2.value.p);
        const auto& hm2=complex_geoms[6].AsHeightmap();
        printf("  Heightmap %d %d\n",hm2.heights.m,hm2.heights.n);
        printf("Press enter to continue\n");
        getchar();

        CapabilityMatrix(simple_geoms,simple_labels,"simple_geometry_stats");
        CapabilityMatrix(complex_geoms,complex_labels,"complex_geometry_stats");
    }
}


int main(int argc,const char** argv)
{
    GenerateCapabilityMatrix();
    return 0;
}
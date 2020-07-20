/******************************************
 * Author: SwarmUS
 * Description: Contains functions  
 * used to control buzz execution through 
 * the ROS ecosystem. 
 ******************************************/

#include "BuzzUtility.hpp"
/*************************************************************************************************/
namespace BuzzUtility {

/*************************************************************************************************/
static buzzvm_t VM = 0;
static char* boFilename = 0;
static uint8_t* boBuffer = 0;
static buzzdebug_t dbgInfo = 0;
static uint8_t robotID = 0;
/*************************************************************************************************/

/*************************************************************************************************/
int setBuzzScript(const char* p_boFilename, const char* p_bdbgFilename, int p_robotID){
    robotID = p_robotID;
    ROS_INFO("Robot ID: %d", robotID);
    FILE* fd = fopen(p_boFilename, "rb");
    if (!fd)
    {
        perror(p_boFilename);
        return 0;
    }
    fseek(fd, 0, SEEK_END);
    size_t bcode_size = ftell(fd);
    rewind(fd);
    boBuffer = (uint8_t*)malloc(bcode_size);
    if (fread(boBuffer, 1, bcode_size, fd) < bcode_size)
    {
        perror(p_boFilename);
        buzzvm_destroy(&VM);
        buzzdebug_destroy(&dbgInfo);
        fclose(fd);
        return 0;
    }
    fclose(fd);

    // Save bytecode file name
    boFilename = strdup(p_boFilename);

    // Reset the Buzz VM
    if (VM)
        buzzvm_destroy(&VM);
    VM = buzzvm_new(robotID);
    // Get rid of debug info
    if (dbgInfo)
        buzzdebug_destroy(&dbgInfo);
    dbgInfo = buzzdebug_new();

    // Read debug information
    if (!buzzdebug_fromfile(dbgInfo, p_bdbgFilename))
    {
        buzzvm_destroy(&VM);
        buzzdebug_destroy(&dbgInfo);
        perror(p_bdbgFilename);
        return 0;
    }
    // Set byte code
    if (buzzvm_set_bcode(VM, boBuffer, bcode_size) != BUZZVM_STATE_READY)
    {
        buzzvm_destroy(&VM);
        buzzdebug_destroy(&dbgInfo);
        ROS_ERROR("%d: Error loading Buzz bytecode", robotID);
        return 0;
    }

    // Execute the global part of the script
    if (buzzvm_execute_script(VM) != BUZZVM_STATE_DONE)
    {
        ROS_ERROR("Error executing global part, VM state : %i", VM->state);
        return 0;
    }
    // Call the Init() function
    if (buzzvm_function_call(VM, "init", 0) != BUZZVM_STATE_READY)
    {
        ROS_ERROR("Error in  calling init, VM state : %i", VM->state);
        return 0;
    }
    // Pop the init return value
    buzzvm_pop(VM);
    // All OK

    // Register base log function in Buzz for ROS
    registerHookFunction("log", BuzzUtility::buzzPrint);
    return 1;
}

/*************************************************************************************************/
void buzzScriptStep() {
    
    if (buzzvm_function_call(VM, "step", 0) != BUZZVM_STATE_READY)
    {
        ROS_ERROR("%s: execution terminated abnormally: %s", boFilename, buzzErrorInfo());
        buzzvm_dump(VM);
    }
    // Pop the step return value
    buzzvm_pop(VM);
}

/*************************************************************************************************/
void buzzScriptDestroy() {
    if (VM->state != BUZZVM_STATE_READY)
    {
        ROS_ERROR("%s: execution terminated abnormally: %s", boFilename, buzzErrorInfo());
        buzzvm_dump(VM);
    }
    buzzvm_function_call(VM, "destroy", 0);
    buzzvm_destroy(&VM);
    free(boFilename);
    buzzdebug_destroy(&dbgInfo);
    ROS_INFO("Script execution stopped.");
}

/*************************************************************************************************/
int buzzScriptDone() {
    return VM->state != BUZZVM_STATE_READY;
}


/*************************************************************************************************/
int getRobotID() {
    return robotID;
}

/*************************************************************************************************/
int getSwarmSize() {
    return (int)buzzdict_size(VM->swarmmembers) + 1;
}

/*************************************************************************************************/
buzzvm_t getVM() {
    return VM;
}

/*************************************************************************************************/
std::string getVMState() {
    std::string state = "Unknown";
    if (VM && VM->strings)
    {
    buzzvm_pushs(VM, buzzvm_string_register(VM, "BVMSTATE", 1));
    buzzvm_gload(VM);
    buzzobj_t obj = buzzvm_stack_at(VM, 1);
    if (obj->o.type == BUZZTYPE_STRING)
        state = std::string(obj->s.value.str);
    else
        state = "Not Available";
    buzzvm_pop(VM);
    }
    return state;
}

/*************************************************************************************************/
const char* buzzErrorInfo() {
  buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(dbgInfo, &VM->pc);
  char* msg;
  if (dbg != NULL)
  {
    asprintf(&msg, "%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n", boFilename, dbg->fname,
             dbg->line, dbg->col, VM->errormsg);
  }
  else
  {
    asprintf(&msg, "%s: execution terminated abnormally at bytecode offset %d: %s\n\n", boFilename, VM->pc, VM->errormsg);
  }
  return msg;
}

/*************************************************************************************************/
std::string compileBuzzScript(std::string p_bzzFilename){
    std::stringstream bzzfile_in_compile;
    std::string path = p_bzzFilename.substr(0, p_bzzFilename.find_last_of("\\/")) + "/";
    std::string name = p_bzzFilename.substr(p_bzzFilename.find_last_of("/\\") + 1);
    name = name.substr(0, name.find_last_of("."));
    bzzfile_in_compile << "bzzc -I " << path << "include/";
    bzzfile_in_compile << " -b " << path << name << ".bo";
    bzzfile_in_compile << " -d " << path << name << ".bdb ";
    bzzfile_in_compile << p_bzzFilename;

    ROS_WARN("Launching buzz compilation: %s", bzzfile_in_compile.str().c_str());

    system(bzzfile_in_compile.str().c_str());
    
    return path + name;
}

/*************************************************************************************************/
int registerHookFunction(const char* p_BuzzFunctionName, buzzvm_funp p_CallbackFunctionPointer) {
    buzzvm_pushs(VM, buzzvm_string_register(VM, p_BuzzFunctionName, 1));
    buzzvm_pushcc(VM, buzzvm_function_register(VM, p_CallbackFunctionPointer));
    buzzvm_gstore(VM);
    return VM->state;
}

int buzzPrint(buzzvm_t vm) {
    std::ostringstream buffer(std::ostringstream::ate);
    for (uint32_t index = 1; index < buzzdarray_size(vm->lsyms->syms); ++index)
    {
        buzzvm_lload(vm, index);
        buzzobj_t o = buzzvm_stack_at(vm, 1);
        buzzvm_pop(vm);
        switch (o->o.type)
        {
        case BUZZTYPE_NIL:
            buffer << " BUZZ - [nil]";
            break;
        case BUZZTYPE_INT:
            buffer << " " << o->i.value;
            break;
        case BUZZTYPE_FLOAT:
            buffer << " " << o->f.value;
            break;
        case BUZZTYPE_TABLE:
            buffer << " [table with " << buzzdict_size(o->t.value) << " elems]";
            break;
        case BUZZTYPE_CLOSURE:
            if (o->c.value.isnative)
            {
            buffer << " [n-closure @" << o->c.value.ref << "]";
            }
            else
            {
            buffer << " [c-closure @" << o->c.value.ref << "]";
            }
            break;
        case BUZZTYPE_STRING:
            buffer << "  " << o->s.value.str;
            break;
        case BUZZTYPE_USERDATA:
            buffer << " [userdata @" << o->u.value << "]";
            break;
        default:
            break;
        }
    }
    ROS_INFO("%s", buffer.str().c_str());
    return buzzvm_ret0(vm);
}

} // namespace buzz_utility


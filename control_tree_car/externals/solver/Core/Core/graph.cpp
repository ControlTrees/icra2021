/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <map>

#include "util.tpp"
#include "array.tpp"
#include "graph.h"

#define DEBUG(x) //x

Graph __NoGraph;
Graph& NoGraph = __NoGraph;

NodeL& NoNodeL=*((NodeL*)NULL);
//Graph& NoGraph=*((Graph*)NULL);

//===========================================================================
//
// annotations to a node while parting; can be used for highlighting and error messages
//

struct ParseInfo {
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  istream::pos_type keys_beg, keys_end;
  istream::pos_type parents_beg, parents_end;
  istream::pos_type value_beg, value_end;
  enum Error { good=0, unknownParent };
  void write(ostream& os) const { os <<'<' <<beg <<',' <<end <<'>'; }
};
stdOutPipe(ParseInfo)

//===========================================================================
//
// retrieving types
//

//-- query existing types
inline Node *reg_findType(const char* key) {
  NodeL types = registry()->getNodesOfType<std::shared_ptr<Type> >();
  for(Node *ti: types) {
    if(rai::String(ti->get<std::shared_ptr<Type> >()->typeId().name())==key) return ti;
    if(ti->matches(key)) return ti;
  }
  return NULL;
}

//===========================================================================
//
// read a value from a stream by looking up available registered types
//

inline Node* readTypeIntoNode(Graph& container, const char* key, std::istream& is) {
  Node *ti = reg_findType(key);
  if(ti) return ti->get<std::shared_ptr<Type> >()->readIntoNewNode(container, is);
  return NULL;
}

//===========================================================================
//
//  Node methods
//

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container)
  : type(_type), value_ptr(_value_ptr), container(_container) {
  CHECK(&container!=&NoGraph, "don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
}

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container, const StringA& _keys, const NodeL& _parents)
  : type(_type), value_ptr(_value_ptr), container(_container), keys(_keys) {
  CHECK(&container!=&NoGraph, "This is a NGraph (NULL) -- don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
  if(_parents.N) for(Node *p: _parents) addParent(p);
}

Node::~Node() {
  if(container.isDoubleLinked) while(parentOf.N) parentOf.last()->removeParent(this);
  if(numChildren) LOG(-2) <<"It is not allowed to delete nodes that still have children";
  while(parents.N) removeParent(parents.last());
  if(this==container.last()) { //great: this is very efficient to remove without breaking indexing
    container.resizeCopy(container.N-1);
  } else {
    container.removeValue(this);
    container.isIndexed=false;//  container.index();
  }
}

void Node::addParent(Node *p) {
  CHECK(p,"you gave me a NULL parent");
//  if(parents.contains(p)) return; //allow the same parent twice!
  parents.append(p);
  p->numChildren++;
  if(container.isDoubleLinked) p->parentOf.append(this);
}

void Node::removeParent(Node *p) {
  if(p==parents.last()) parents.removeLast(); else parents.removeValue(p);
  CHECK(p->numChildren,"");
  p->numChildren--;
  if(container.isDoubleLinked) p->parentOf.removeValue(this);
}

void Node::swapParent(uint i, Node *p) {
  CHECK(p,"you gave me a NULL parent");
  parents(i)->numChildren--;
  if(container.isDoubleLinked) parents(i)->parentOf.removeValue(this);
  parents(i) = p;
  parents(i)->numChildren++;
  if(container.isDoubleLinked) parents(i)->parentOf.append(this);
}

bool Node::matches(const char *key) {
  for(const rai::String& k:keys) if(k==key) return true;
  return false;
}

bool Node::matches(const StringA &query_keys) {
  for(const rai::String& k:query_keys) {
    if(!matches(k)) return false;
  }
  return true;
}

void Node::write(std::ostream& os, bool pythonMode) const {
  if(!container.isIndexed) container.index();
  
  //-- write keys
  if(pythonMode)
    keys.write(os, " ", "", "\"\"");
  else
    keys.write(os, " ", "", "\0\0");

  //-- write parents
  if(parents.N) {
    //    if(keys.N) os <<' ';
    os <<'(';
    for_list(Node, it, parents) {
      if(it_COUNT) os <<' ';
      if(it->keys.N && it->keys.last().N) {
        os <<it->keys.last();
      } else { //relative numerical reference
        os <<(int)it->index - (int)index;
      }
    }
    os <<')';
  }
  
  //-- write value
  if(isGraph()) {
    os <<": ";
    graph().write(os, ", ", "{}");
  } else if(isOfType<NodeL>()) {
    os <<":(";
    for(Node *it: (*getValue<NodeL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(isOfType<rai::String>()) {
    if(pythonMode){
      os <<":\"" <<*getValue<rai::String>() <<'"';
    }else{
      const rai::String& str = *getValue<rai::String>();
      char c=str(0);
      if((c>='a'&& c<='z') || (c>='A'&& c<='Z') ) os <<":" <<str;
      else os <<":\"" <<str <<'"';
    }
  } else if(isOfType<rai::FileToken>()) {
    os <<":'" <<getValue<rai::FileToken>()->name <<'\'';
  } else if(isOfType<arr>()) {
    os <<": "; getValue<arr>()->write(os, ", ", NULL, "[]");
  } else if(isOfType<intA>()) {
    os <<": "; getValue<intA>()->write(os, ", ", NULL, "[]");
  } else if(isOfType<intAA>()) {
    os <<": "; getValue<intAA>()->write(os, ", ", NULL, "[]");
  } else if(isOfType<uintA>()) {
    os <<": "; getValue<uintA>()->write(os, ", ", NULL, "[]");
  } else if(isOfType<StringA>()) {
    os <<": [";
    for(const rai::String& s:get<StringA>()) os <<'\"' <<s <<"\", ";
    os <<']';
  } else if(isOfType<double>()) {
    os <<": " <<*getValue<double>();
  } else if(isOfType<int>()) {
    os <<": " <<*getValue<int>();
  } else if(isOfType<uint>()) {
    os <<": " <<*getValue<uint>();
  } else if(isOfType<bool>()) {
    if(*getValue<bool>()) os <<": true"; else os <<": false";
  } else if(isOfType<Type*>()) {
    os <<": "; get<Type*>()->write(os);
  } else {
    Node *it = reg_findType(type.name());
    if(it && it->keys.N>1) {
      os <<":<" <<it->keys(1) <<' ';
      writeValue(os);
      os <<'>';
    } else {
      os <<": ";
      writeValue(os);
    }
  }
}

NodeInitializer::NodeInitializer(const char* key) {
  n = G.newNode<bool>(true);
  n->keys.append(STRING(key));
}

NodeInitializer::NodeInitializer(const char* key, const char* stringValue) {
  n = G.newNode<rai::String>(STRING(stringValue));
  n->keys.append(STRING(key));
}

//===========================================================================
//
//  Graph methods
//

Graph::Graph() : isNodeOfGraph(NULL), pi(NULL), ri(NULL) {
}

Graph::Graph(const char* filename): Graph() {
  read(rai::FileToken(filename).getIs());
}

Graph::Graph(istream& is) : Graph() {
  read(is);
}

Graph::Graph(const std::map<std::string, std::string>& dict) : Graph() {
  appendDict(dict);
}

Graph::Graph(std::initializer_list<NodeInitializer> list) : Graph() {
  for(const NodeInitializer& ni:list) newNode(ni);
}

//Graph::Graph(std::initializer_list<const char*> list) : Graph(){
//  for(const char* s:list){
//    rai::String str(s);
//    Graph &g=newSubgraph();
//    g.read(str);
//  }
//}

Graph::Graph(const Graph& G) : Graph() {
  *this = G;
}

Graph::~Graph() {
  clear();
}

bool Graph::operator!() const {
  return this==&__NoGraph;
}

void Graph::clear() {
  if(ri) { delete ri; ri=NULL; }
  if(pi) { delete pi; pi=NULL; }
  DEBUG(checkConsistency();)
      if(!isNodeOfGraph) { //this is not a subgraph; save to delete connections in batch -> faster
    NodeL all = getAllNodesRecursively();
    for(Node *n:all) {
      n->parents.clear();
      n->numChildren=0;
      n->parentOf.clear();
      n->keys.clear();
    }
    DEBUG(checkConsistency();)
  }
  //delete all subgraphs first to remove potential children
  for(Node *n:*this) if(n->isGraph()) n->graph().clear();
  while(N) {
    Node **n = NodeL::p+N-1; //last
    if(!isDoubleLinked) while((*n)->numChildren) { n--; CHECK_GE(n, p,"can't find a node without children"); }
    delete *n;
  }
  isIndexed=true;
}

Graph& Graph::newNode(const NodeInitializer& ni) {
  Node *clone = ni.n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
  for(const rai::String& s:ni.parents) {
    Node *p = findNode({s}, true, false);
    CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
    clone->addParent(p);
  }
  return *this;
}

Graph& Graph::newSubgraph(const StringA& keys, const NodeL& parents, const Graph& x) {
  Node_typed<Graph>* n = newNode<Graph>(keys, parents, Graph());
  DEBUG(CHECK(n->value.isNodeOfGraph && &n->value.isNodeOfGraph->container==this,""))
      if(!!x) n->value.copy(x);
  n->value.isDoubleLinked = isDoubleLinked;
  return n->value;
}

Node_typed<int>* Graph::newNode(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0; i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return newNode<int>({STRING(NodeL::N)}, parents, 0);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict) {
  for(const std::pair<std::string,std::string>& p:dict) {
    Node *n = readNode(STRING(':'<<p.second), false, false, rai::String(p.first));
    if(!n) RAI_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::findNode(const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNode(keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) {
    ret = n->graph().findNode(keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

Node* Graph::findNodeOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->type==type && n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNodeOfType(type, keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) {
    ret = n->graph().findNodeOfType(type, keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

NodeL Graph::findNodes(const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append(isNodeOfGraph->container.findNodes(keys, true, false));
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append(n->graph().findNodes(keys, false, true));
  return ret;
}

NodeL Graph::findNodesOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->type==type && n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append(isNodeOfGraph->container.findNodesOfType(type, keys, true, false));
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append(n->graph().findNodesOfType(type, keys, false, true));
  return ret;
}

//Node* Graph::getNode(const char *key) const {
//  for(Node *n: (*this)) if(n->matches(key)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(key);
//  return NULL;
//}

//Node* Graph::getNode(const StringA &keys) const {
//  for(Node *n: (*this)) if(n->matches(keys)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(keys);
//  return NULL;
//}

//NodeL Graph::getNodes(const StringA &keys) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
//  return ret;

//}

//NodeL Graph::getNodes(const char* key) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(key)) ret.append(n);
//  return ret;
//}

Node* Graph::getEdge(Node *p1, Node *p2) const {
  if(p1->parentOf.N < p2->parentOf.N) {
    for(Node *i:p1->parentOf) {
      if(p2->parentOf.findValue(i)!=-1) return i;
    }
  } else {
    for(Node *i:p2->parentOf) {
      if(p1->parentOf.findValue(i)!=-1) return i;
    }
  }
  return NULL;
}

Node* Graph::getEdge(const NodeL& parents) const {
  CHECK(parents.N>0,"");
  //grap 'sparsest' parent:
  uint minSize = this->N;
  Node *sparsestParent = NULL;
  for(Node *p:parents) if(p->parentOf.N<minSize) { sparsestParent=p; minSize=p->parentOf.N; }
  if(!sparsestParent) {
    for(Node *e:*this) if(e->parents==parents) return e;
  } else {
    for(Node *e:sparsestParent->parentOf) if(&e->container==this) {
      if(e->parents==parents) return e;
    }
  }
  return NULL;
}

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node *n: (*this)) if(n->parents.N==deg) ret.append(n);
  return ret;
}

NodeL Graph::getAllNodesRecursively() const {
  NodeL ret = *this;
  NodeL below;
  for(Node *n:ret) if(n->isGraph()) below.append(n->graph().getAllNodesRecursively());
  ret.append(below);
  return ret;
}

Node* Graph::edit(Node *ed) {
  NodeL KVG = findNodesOfType(ed->type, ed->keys);
  //CHECK_LE(KVG.N, 1, "can't edit into multiple nodes yet");
  if(!KVG.N) { //nothing to merge, append
    if(&ed->container!=this) {
      if(!isIndexed) index();
      if(!ed->container.isIndexed) ed->container.index();
      Node *it = ed->newClone(*this);
      for(uint i=0; i<it->parents.N; i++) {
        it->swapParent(i, elem(it->parents(i)->index));
      }
    }
    return ed;
  }
  
  uint edited=0;
  for(Node *n : KVG) if(n!=ed) {
    CHECK(ed->type == n->type, "can't edit/merge nodes of different types!");
    for(Node *p:ed->parents) n->addParent(p);
    if(n->isGraph()) { //merge the KVGs
      n->graph().edit(ed->graph());
    } else { //overwrite the value
      n->copyValue(ed);
    }
    edited++;
  }
  if(!edited){
    RAI_MSG("no nodes edited!");
  }
  if(&ed->container==this) { delete ed; ed=NULL; }
  return NULL;
}

void Graph::collapse(Node* a, Node* b){
  NodeL ab={a,b}, ba={b,a};
//  cout <<"collapsing " <<a->keys.first() <<' ' <<b->keys.first() <<endl;
//  cout <<"collapsing " <<*a <<listString(a->parentOf) <<" and " <<*b <<listString(b->parentOf) <<endl;
//  a->keys.first() <<'_' <<b->keys.first();
  for(Node *ch:a->parentOf) if(ch->parents==ab || ch->parents==ba) delete ch;
  NodeL b_parentOf = b->parentOf;
  for(Node *ch:b_parentOf){
    for(Node*& p:ch->parents) if(p==b){
      p=a;
      b->parentOf.removeValue(ch);
      b->numChildren--;
      a->parentOf.prepend(ch);
      a->numChildren++;
    }
  }
//  cout <<"... becomes " <<*a <<listString(a->parentOf) <<" and " <<*b <<listString(b->parentOf) <<endl;
//  checkConsistency();
  delete b;
}

void Graph::copy(const Graph& G, bool appendInsteadOfClear, bool enforceCopySubgraphToNonsubgraph) {
  DEBUG(G.checkConsistency();)
      if(!G.isIndexed) HALT("can't copy non-indexed graph");
  
  CHECK(this!=&G, "Graph self copy -- never do this");
  
  if(!enforceCopySubgraphToNonsubgraph) {
    if(G.isNodeOfGraph && !this->isNodeOfGraph) {
      HALT("Typically you should not copy a subgraph into a non-subgraph (or call the copy operator with a subgraph).\
           Use 'newSubgraph' instead\
           If you still want to do it you need to ensure that all node parents are declared, and then enforce it by setting 'enforceCopySubgraphToNonsubgraph'");
    }
  } else {
    if(this->isNodeOfGraph) {
      HALT("You set 'enforceCopySubgraphToNonsubgraph', but this is not a Nonsubgraph");
    }
  }
  
  //-- first delete existing nodes
  if(!appendInsteadOfClear) clear();
  uint indexOffset=N;
  NodeL newNodes;
  
  //-- if either is a subgraph, ensure they're a subgraph of the same -- over restrictive!!
  //  if(isNodeOfGraph || G.isNodeOfGraph){
  //    CHECK(&isNodeOfGraph->container==&G.isNodeOfGraph->container,"is already subgraph of another container!");
  //  }

  //-- first, just clone nodes with their values -- 'parents' still point to the origin nodes
  for(Node *n:G) {
    Node *newn=NULL;
    if(n->isGraph()) {
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all nodes are created
      newn = this->newSubgraph(n->keys, n->parents).isNodeOfGraph;
    } else {
      newn = n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
    }
    newNodes.append(newn);
  }
  
  //-- the new nodes are not parent of anybody yet
#ifndef RAI_NOCHECK
  for(Node *n:newNodes) CHECK(n->numChildren==0 && n->parentOf.N==0,"");
#endif
  
  //-- now copy subgraphs
  for(Node *n:newNodes) if(n->isGraph()) {
    n->graph().copy(G.elem(n->index-indexOffset)->graph()); //you can only call the operator= AFTER assigning isNodeOfGraph
  }

  //-- now rewire parental links
  for(Node *n:newNodes) {
    for(uint i=0; i<n->parents.N; i++) {
      Node *p=n->parents(i); //the parent in the origin graph
      Node *newp=NULL;
      if(isChildOfGraph(p->container)) continue;
      if(&p->container==&G) { //parent is directly in G, no need for complicated search
        newp = newNodes.elem(p->index);  //the true parent in the new graph
      } else {
        const Graph *newg=this, *oldg=&G;
        while(&p->container!=oldg) { //find the container while iterating backward also in the newG
          CHECK(oldg->isNodeOfGraph,"");
          CHECK(newg->isNodeOfGraph,"");
          newg = &newg->isNodeOfGraph->container;
          oldg = &oldg->isNodeOfGraph->container;
        }
        CHECK_EQ(newg->N, oldg->N,"different size!!\n" <<*newg <<"**\n" <<*oldg);
        CHECK_EQ(p, oldg->elem(p->index),""); //we found the parent in oldg
        newp = newg->elem(p->index);     //the true parent in the new graph
      }
      n->swapParent(i, newp);
    }
  }
  
  DEBUG(this->checkConsistency());
  DEBUG(G.checkConsistency());
}

void Graph::read(std::istream& is, bool parseInfo) {
  if(parseInfo) getParseInfo(NULL).beg=is.tellg();
  rai::String namePrefix;
  for(;;) {
    DEBUG(checkConsistency());
    char c=rai::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node *n = readNode(is, false, parseInfo);
    if(n->keys.N==1 && n->keys.first()=="Quit") {
      delete n; n=NULL;
    }
    if(!n) break;
    if(n->keys.N==1 && n->keys.last()=="Include") {
      uint Nbefore = N;
      read(n->get<rai::FileToken>().getIs(true), parseInfo);
      if(namePrefix.N){
        for(uint i=Nbefore;i<N;i++) elem(i)->keys.last().prepend(namePrefix);
        namePrefix.clear();
      }
      n->get<rai::FileToken>().cd_start();
      delete n; n=NULL;
    } else if(n->keys.N==1 && n->keys.last()=="Prefix") {
      namePrefix = n->get<rai::String>();
      delete n; n=NULL;
    } else if(n->keys.N==1 && n->keys.last()=="ChDir") {
      n->get<rai::FileToken>().cd_file();
    } else if(n->keys.N>0 && n->keys.first()=="Delete") {
      n->keys.remove(0);
      NodeL dels = getNodes(n->keys);
      for(Node* d: dels) { delete d; d=NULL; }
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();
  
  DEBUG(checkConsistency());

  //-- merge all Mege keys
  NodeL edits = getNodes("Edit");
  for(Node *ed:edits) {
    CHECK_EQ(ed->keys.first(), "Edit" , "an edit node needs Edit as first key");
    ed->keys.remove(0);
    edit(ed);
  }
  
  DEBUG(checkConsistency();)

  //-- delete all ChDir nodes in reverse order
  for(uint i=N; i--;) {
    Node *n=elem(i);
    if(n->keys.N==1 && n->keys(0)=="ChDir") {
      n->get<rai::FileToken>().cd_start();
      delete n; n=NULL;
    }
  }

  index();
}

void writeFromStream(std::ostream& os, std::istream& is, istream::pos_type beg, istream::pos_type end) {
  istream::pos_type here=is.tellg();
  is.seekg(beg);
  char c;
  for(uint i=end-beg; i--;) {
    is.get(c);
    os <<c;
  }
  is.seekg(here);
}

#define PARSERR(x, pinfo) { \
  cerr <<"[[error in parsing Graph file (line=" <<rai::lineCount <<"): " <<x <<":\n  \""; \
  writeFromStream(cerr, is, pinfo.beg, is.tellg()); \
  cerr <<"<<<\"  ]]" <<endl; \
  is.clear(); }

//  if(node) cerr <<"  (node='" <<*node <<"')" <<endl;

void readNodeParents(Graph &G, std::istream& is, NodeL& parents, ParseInfo& pinfo){
  rai::String str;
  pinfo.parents_beg=is.tellg();
  for(uint j=0;; j++) {
    if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
    Node *e = G.findNode({str}, true, false); //important: recurse up
    if(e) { //sucessfully found
      parents.append(e);
      pinfo.parents_end=is.tellg();
    } else { //this element is not known!!
      int rel=0;
      str >>rel;
      if(rel<0 && (int)G.N+rel>=0) { //check if this is a negative integer
        e=G.elem(G.N+rel);
        parents.append(e);
        pinfo.parents_end=is.tellg();
      } else {
        PARSERR("unknown " <<j <<". parent '" <<str <<"'", pinfo);
        rai::skip(is, NULL, ")", false);
      }
    }
  }
  rai::parse(is, ")");
}

Node* Graph::readNode(std::istream& is, bool verbose, bool parseInfo, rai::String prefixedKey) {
  rai::String str;
  
  ParseInfo pinfo;
  pinfo.beg=is.tellg();
  
  if(verbose) { cout <<"\nNODE (line="<<rai::lineCount <<")"; }
  
  //-- read keys
  StringA keys;
  if(!prefixedKey.N) {
    rai::skip(is," \t\n\r");
    pinfo.keys_beg=is.tellg();
    for(;;) {
      if(!str.read(is, " \t", " \t\n\r,;([{}=:!\'", false)) break;
      if(str(0)=='"' && str(-1)=='"') str = str.getSubString(1,-2);
      keys.append(str);
      pinfo.keys_end=is.tellg();
    }
  } else {
    keys.append(prefixedKey);
  }
  DEBUG(checkConsistency());

  if(verbose) { cout <<" keys:" <<keys <<flush; }
  
  //-- read parents
  NodeL parents;
  char c=rai::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    readNodeParents(*this, is, parents, pinfo);
    c=rai::getNextChar(is," \t");
  }
  DEBUG(checkConsistency());

  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }
  
  //-- read value
  Node *node=NULL;
  pinfo.value_beg=(long int)is.tellg()-1;
  if(c=='=' || c==':' || c=='{' || c=='[' || c=='<' || c=='!' || c=='\'') {
    if(c=='=' || c==':') c=rai::getNextChar(is," \t");
    if((c>='a' && c<='z') || (c>='A' && c<='Z') || c=='_') { //rai::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") node = newNode<bool>(keys, parents, true);
      else if(str=="false") node = newNode<bool>(keys, parents, false);
      else node = newNode<rai::String>(keys, parents, str);
    } else if(rai::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse the double number", pinfo);
      node = newNode<double>(keys, parents, d);
    } else switch(c) {
      case '!': { //boolean false
        node = newNode<bool>(keys, parents, false);
      } break;
      case '\'': { //rai::FileToken
        str.read(is, "", "\'", true);
        try {
          node = newNode<rai::FileToken>(keys, parents, rai::FileToken(str, false));
//          node->get<rai::FileToken>().getIs();  //creates the ifstream and might throw an error
        } catch(...) {
          delete node; node=NULL;
          PARSERR("file " <<str <<" does not exist -> converting to string!", pinfo);
          node = newNode<rai::String>(keys, parents, str);
        }
      } break;
      case '\"': { //rai::String
        str.read(is, "", "\"", true);
        node = newNode<rai::String>(keys, parents, str);
      } break;
        case '[': { //arr or StringA
          char c2=rai::getNextChar(is," \t");
          if(c2=='"') { //StringA
            is.putback(c2);
            is.putback(c);
            StringA strings;
            rai::String::readSkipSymbols=",\"";
            rai::String::readStopSymbols="\"";
            rai::String::readEatStopSymbol = 1;
            is >>strings;
            rai::String::readSkipSymbols = " \t";
            rai::String::readStopSymbols = "\n\r";
            rai::String::readEatStopSymbol = 1;
            node = newNode<StringA>(keys, parents, strings);
          } else if((c2>='a' && c2<='z') || (c2>='A' && c2<='Z')){ //StringA}
              is.putback(c2);
              is.putback(c);
              StringA strings;
              rai::String::readStopSymbols=" \n\t]";
              rai::String::readEatStopSymbol = 0;
              is >>strings;
              rai::String::readStopSymbols = "\n\r";
              rai::String::readEatStopSymbol = 1;
              node = newNode<StringA>(keys, parents, strings);
          } else {
            is.putback(c2);
            is.putback(c);
            arr reals;
            is >>reals;
            node = newNode<arr>(keys, parents, reals);
          }
        } break;
        case '<': { //any type parser
#if 1
        str.read(is, "", ">", true);
        node = newNode<rai::String>(keys, parents, str);
#else
        str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
        //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
        //        node = readTypeIntoNode(*this, str, is);
        if(!node) {
          is.clear();
          rai::String substr;
          substr.read(is,"",">",false);
          //          PARSERR("could not parse value of type '" <<str <<"' -- no such type has been registered; converting this to string: '"<<substr<<"'", pinfo);
          str = STRING('<' <<str <<' ' <<substr <<'>');
          node = newNode<rai::String>(keys, parents, str);
        } else {
          node->keys = keys;
          node->parents = parents;
        }
        rai::parse(is, ">");
#endif
      } break;
      case '(': { // set of parent nodes
        NodeL par;
        readNodeParents(*this, is, par, pinfo);
        node = newNode<NodeL>(keys, parents, par);
      } break;
      case '{': { // sub graph
        Graph& subgraph = this->newSubgraph(keys, parents);
        subgraph.read(is);
        rai::parse(is, "}");
        node = subgraph.isNodeOfGraph;
      } break;
      default: { //error
        is.putback(c);
        PARSERR("unknown value indicator '" <<c <<"'", pinfo);
        return NULL;
      }
    }
  } else { //no ':' or '{' -> boolean
    is.putback(c);
    node = newNode<bool>(keys, parents, true);
  }
  if(node) pinfo.value_end=is.tellg();
  pinfo.end=is.tellg();
  DEBUG(checkConsistency();)

      if(parseInfo && node) node->container.getParseInfo(node) = pinfo;
  
  if(verbose) {
    if(node) { cout <<" value:"; node->writeValue(cout); cout <<" FULL:"; node->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }
  
  if(!node) {
    cerr <<"FAILED reading node with keys ";
    keys.write(cerr, " ", NULL, "()");
    cerr <<" and parents ";
    listWrite(parents,cerr," ","()");
    cerr <<endl;
  }
  
  //eat the next , or ;
  c=rai::getNextChar(is," \n\r\t");
  if(c==',' || c==';') {} else is.putback(c);
  
  return node;
}

#undef PARSERR

void Graph::write(std::ostream& os, const char *ELEMSEP, const char *BRACKETS) const {
  uint BRACKETSlength=0;
  if(BRACKETS) BRACKETSlength=strlen(BRACKETS);
  for(uint b=0;b<BRACKETSlength/2;b++) os <<BRACKETS[b];
  for(uint i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) elem(i)->write(os); else os <<"<NULL>"; }
  for(uint b=BRACKETSlength/2; b<BRACKETSlength; b++) os <<BRACKETS[b];
  os <<std::flush;
}

void Graph::writeParseInfo(std::ostream& os) {
  os <<"GRAPH " <<getParseInfo(NULL) <<endl;
  for(Node *n:*this)
    os <<"NODE '" <<*n <<"' " <<getParseInfo(n) <<endl;
}

void Graph::displayDot(Node *highlight) {
  if(highlight) {
    CHECK(&highlight->container==this,"");
    writeDot(FILE("z.dot"), false, false, 0, highlight->index, true);
  } else {
    writeDot(FILE("z.dot"), false, false, 0, -1, true);
  }
  int r;
  r = system("dot -Tpdf z.dot > z.pdf");  if(r) LOG(-1) <<"could not startup dot";
  r = system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince";
}

void Graph::writeHtml(std::ostream& os, std::istream& is) {
  char c;
  long int g=getParseInfo(NULL).beg;
  is.seekg(g);
#define GO { is.get(c); if(c=='\n') os <<"<br>" <<endl; else os <<c; g++; }
  for(Node *n:list()) {
    ParseInfo& pinfo=getParseInfo(n);
    while(g<pinfo.keys_beg) GO
        os <<"<font color=\"0000ff\">";
    while(g<pinfo.keys_end) GO
        os <<"</font>";
    while(g<pinfo.parents_beg)GO
        os <<"<font color=\"00ff00\">";
    while(g<pinfo.parents_end)GO
        os <<"</font>";
    while(g<pinfo.value_beg)GO
        os <<"<font color=\"ff0000\">";
    while(g<pinfo.value_end)GO
        os <<"</font>";
  }
  while(g<getParseInfo(NULL).end)GO
    #undef GO
}

void Graph::writeDot(std::ostream& os, bool withoutHeader, bool defaultEdges, int nodesOrEdges, int focusIndex, bool subGraphsAsNodes) {
  if(!withoutHeader) {
    os <<"digraph G{" <<endl;
    os <<"graph [ rankdir=\"LR\", ranksep=0.05";
    if(hasRenderingInfo(NULL)) os <<' ' <<getRenderingInfo(NULL).dotstyle;
    os << " ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
    index(true);
  } else {
    if(!isIndexed) index();
  }
  for(Node *n: list()) {
    if(hasRenderingInfo(n) && getRenderingInfo(n).skip) continue;
    rai::String label;
    if(n->keys.N) {
      bool newline=false;
      for(rai::String& k:n->keys) {
        if(newline) label <<"\\n";
        label <<k;
        newline=true;
      }
    } else {
//      if(n->parents.N) {
//        label <<"(" <<n->parents(0)->keys.last();
//        for(uint i=1; i<n->parents.N; i++) label <<' ' <<n->parents(i)->keys.last();
//        label <<")";
//      }
    }
    if(label.N) label <<"\\n";
    n->writeValue(label);
    
    rai::String shape;
    if(n->keys.contains("box")) shape <<", shape=box"; else shape <<", shape=ellipse";
    if(focusIndex==(int)n->index) shape <<", color=red";
    if(hasRenderingInfo(n)) shape <<' ' <<getRenderingInfo(n).dotstyle;
    
    if(defaultEdges && n->parents.N==2) { //an edge
      os <<n->parents(0)->index <<" -> " <<n->parents(1)->index <<" [ label=\"" <<label <<"\" ];" <<endl;
    } else {
      if(n->isGraph()) {
        bool graphNodesInside=false;
        Graph &nG = n->graph();
        for(Node *c:nG) if(c->parents.N || c->isGraph()) graphNodesInside=true;
        if(!subGraphsAsNodes && graphNodesInside){
          os <<"subgraph cluster_" <<n->index <<" { " /*<<" rank=same"*/ <<endl;
          os <<n->index <<" [ label=\"" <<label <<"\" shape=box ];" <<endl;
          n->graph().writeDot(os, true, defaultEdges, +1);
          os <<"}" <<endl;
          n->graph().writeDot(os, true, defaultEdges, -1);
        }else{
          label <<'\n' <<n->graph();
          for(uint i=0;i<label.N;i++) if(label(i)=='"') label(i)='\'';
          os <<n->index <<" [ label=\"" <<label <<"\" shape=box ];" <<endl;
        }
      } else { //normal node
        if(nodesOrEdges>=0) {
          os <<n->index <<" [ label=\"" <<label <<'"' <<shape <<" ];" <<endl;
        }
      }
      if(nodesOrEdges<=0) {
        for_list(Node, pa, n->parents) {
          if(hasRenderingInfo(pa) && getRenderingInfo(pa).skip) continue;
          //              if(pa->index<n->index)
          os <<pa->index <<" -> " <<n->index <<" [ ";
          //              else
          //                  os <<n->index <<" -> " <<pa->index <<" [ ";
          os <<"label=" <<pa_COUNT;
          os <<" ];" <<endl;
        }
      }
    }
  }
  if(!withoutHeader) {
    os <<"}" <<endl;
    index(false);
  }
}

void Graph::sortByDotOrder() {
  uintA perm;
  perm.setStraightPerm(N);
  for_list(Node, it, list()) {
    if(it->isGraph()) {
      double *order = it->graph().find<double>("dot_order");
      if(!order) { RAI_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Node, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Node* n) {
  if(!pi) pi=new ArrayG<ParseInfo>(*this);
  return pi->operator()(n);
  //  if(pi.N!=N+1){
  //    listResizeCopy(pi, N+1);
  //    pi(0)->node=NULL;
  //    for(uint i=1;i<pi.N;i++) pi(i)->node=elem(i-1);
  //  }
  //  if(!n) return *pi(0);
  //  return *pi(n->index+1);
}

RenderingInfo& Graph::getRenderingInfo(Node* n) {
  CHECK(!n || &n->container==this,"");
#if 1
  if(!ri) ri=new ArrayG<RenderingInfo>(*this);
  return ri->operator()(n);
#else
  if(ri.N!=N+1) {
    ri.resizeCopy(N+1); //listResizeCopy(ri, N+1);
    //    ri.elem(0)->node=NULL;
    //    for(uint i=1;i<ri.N;i++) ri.elem(i)->node=elem(i-1);
  }
  if(!n) return ri.elem(0);
  return ri.elem(n->index+1);
#endif
}

const Graph* Graph::getRootGraph() const {
  const Graph* g=this;
  for(;;) {
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
  }
  return g;
}

bool Graph::isChildOfGraph(const Graph& G) const {
  const Graph* g=this;
  for(;;) {
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
    if(g==&G) return true;
  }
  return false;
}

bool Graph::checkConsistency() const {
  uint idx=0;
  
#if 0 //this is expensive: fill all the parentsOf lists
  NodeL ALL = getAllNodesRecursively();
  if(!isDoubleListed) {
    for(Node *n: ALL) n->parentOf.clear();
    for(Node *n: ALL) for(Node *p:n->parents) p->parentOf.append(n);
  }
  for(Node *n: ALL) CHECK_EQ(n->numChildren, n->parentOf.N, "");
#endif
  
  for(Node *node: *this) {
    CHECK_EQ(&node->container, this, "");
    if(isIndexed) CHECK_EQ(node->index, idx, "");
    if(isDoubleLinked) {
      CHECK_EQ(node->numChildren, node->parentOf.N, "");
#ifndef RAI_NOCHECK
      for(Node *j: node->parents)  CHECK(j->parentOf.findValue(node) != -1,"");
      for(Node *j: node->parentOf) CHECK(j->parents.findValue(node) != -1,"");
#endif
    }
    for(Node *parent: node->parents) if(&parent->container!=this) {
      //check that parent is contained in a super-graph of this
      const Graph *parentGraph = this;
      const Node *parentGraphNode;
      while(&parent->container!=parentGraph) {
        //we need to descend one more
        parentGraphNode = parentGraph->isNodeOfGraph;
        CHECK(parentGraphNode,"there is no more supergraph to find the parent");
        parentGraph = &parentGraphNode->container;
      }
      //check sorting
      //      CHECK(parent->index < parentGraphNode->index,"subnode refers to parent that sorts below the subgraph");
    } else {
      //      CHECK(parent->index < node->index,"node refers to parent that sorts below the node");
    }
    if(node->isGraph()) {
      Graph& G = node->graph();
      CHECK_EQ(G.isNodeOfGraph, node, "");
      G.checkConsistency();
    }
    idx++;
  }
  return true;
}

uint Graph::index(bool subKVG, uint start) {
  uint idx=start;
  for(Node *it: list()) {
    it->index=idx;
    idx++;
    if(it->isGraph()) {
      Graph& G=it->graph();
      if(subKVG) idx = G.index(true, idx);
      else G.index(false, 0);
    }
  }
  isIndexed=true;
  return idx;
}

bool operator==(const Graph& A, const Graph& B) {
  if(A.N!=B.N) return false;
  for(uint i=0; i<A.N; i++) {
    Node *a = A(i), *b = B(i);
    if(a->index!=b->index) return false;
    if(a->keys!=b->keys) return false;
    if(a->parents.N!=b->parents.N) return false;
    for(uint j=0; j<a->parents.N; j++) if(a->parents(j)->index!=b->parents(j)->index) return false;
    if(a->type!=b->type) return false;
    if(!a->hasEqualValue(b)) return false;
  }
  return true;
}

//===========================================================================

NodeL neighbors(Node* it) {
  NodeL N;
  for(Node *e:it->parentOf) {
    for(Node *n:e->parents) if(n!=it) N.setAppend(n);
  }
  return N;
}

int distance(NodeL A, NodeL B){
  CHECK(A.N, "");
  CHECK(B.N, "");
  Graph& G=A.first()->container;
  CHECK_EQ(&B.first()->container, &G, "");

  boolA doneA(G.N), doneB(G.N);
  doneA.setZero();
  doneB.setZero();
  NodeL fringeA = A;
  NodeL fringeB = B;
  int D=0;
  for(Node *a:fringeA) doneA(a->index) = true;
  for(Node *b:fringeB){ if(doneA(b->index)) return D; doneB(b->index) = true; }
  for(;;){
    D++;
    NodeL newA;
    for(Node *a:fringeA){
      NodeL neighA = neighbors(a);
      for(Node *n:neighA){
        if(doneB(n->index)) return D;
        if(!doneA(n->index)){ newA.append(n); doneA(n->index)=true; }
      }
    }
    D++;
    NodeL newB;
    for(Node *b:fringeB){
      NodeL neighB = neighbors(b);
      for(Node *n:neighB){
        if(doneA(n->index)) return D;
        if(!doneB(n->index)){ newB.append(n); doneB(n->index)=true; }
      }
    }
    if(!newA.N && !newB.N) break; //failure
    fringeA = newA;
    fringeB = newB;
  }
  return -1;
}


//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<Graph> registry;

struct RegistryInitializer {
  Mutex lock;
//  Graph cfgParameters;
  RegistryInitializer() {
    int n;
    for(n=1; n<rai::argc; n++) {
      if(rai::argv[n][0]=='-') {
        rai::String key(rai::argv[n]+1);
        if(n+1<rai::argc && rai::argv[n+1][0]!='-') {
          rai::String value;
          value <<':' <<rai::argv[n+1];
          registry()->readNode(value, false, false, key);
          n++;
        } else {
          registry()->newNode<bool>({key}, {}, true);
        }
      } else {
        RAI_MSG("non-parsed cmd line argument:" <<rai::argv[n]);
      }
    }
    
    rai::String cfgFileName="rai.cfg";
    if(registry()()["cfg"]) cfgFileName = registry()->get<rai::String>("cfg");
    LOG(3) <<"opening config file '" <<cfgFileName <<"'";
    ifstream fil;
    fil.open(cfgFileName);
    if(fil.good()) {
      fil >>registry();
    } else {
      LOG(3) <<" - failed";
    }
    
  }
  ~RegistryInitializer() {
  }
};

Singleton<RegistryInitializer> registryInitializer;

bool getParameterFromGraph(const std::type_info& type, void* data, const char* key) {
  registryInitializer()();
  Node *n = registry()->findNodeOfType(type, {key});
  if(n) {
    n->copyValueInto(data);
    return true;
  } else {
    n = registry()->findNode({key});
    if(n && n->isOfType<double>()) {
      if(type==typeid(int)) { *((int*)data) = (int)n->get<double>(); return true; }
      if(type==typeid(uint)) { *((uint*)data) = (uint)n->get<double>(); return true; }
      if(type==typeid(bool)) { *((bool*)data) = (bool)n->get<double>(); return true; }
    }
    if(n && n->isOfType<rai::String>()) {
      NIY;
      //      n->get<rai::String>() >>x;
    }
  }
  return false;
}

//===========================================================================

RUN_ON_INIT_BEGIN(graph)
NodeL::memMove=true;
GraphEditCallbackL::memMove=true;
RUN_ON_INIT_END(graph)
